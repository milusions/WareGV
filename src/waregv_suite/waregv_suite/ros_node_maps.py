"""Map directory resolution, discovery, archive and disk-save helpers.

Implemented as a mixin so it can be combined with the navigation
mixin to build the full WareGVBridgeNode.
"""
import io
import os
import pathlib
import re
import zipfile
from typing import Any, Dict, List, Optional, Tuple


class MapHelpersMixin:
    # --- directory discovery ---------------------------------------------

    def _candidate_map_directories(self) -> List[pathlib.Path]:
        candidates: List[pathlib.Path] = []

        env_dir = os.environ.get("WAREGV_MAP_DIRECTORY", "").strip()
        if env_dir:
            candidates.append(pathlib.Path(env_dir).expanduser())

        home = pathlib.Path.home()
        candidates.extend(
            [
                home / "waregv" / "waregv_ws" / "src" / "waregv_mapping" / "maps",
                home / "waregv" / "waregv_ws" / "install" / "waregv_mapping" / "share" / "waregv_mapping" / "maps",
                home / "waregv_maps",
                pathlib.Path.cwd() / "maps",
            ]
        )

        ros_workspace = os.environ.get("WAREGV_WORKSPACE", "").strip()
        if ros_workspace:
            ws = pathlib.Path(ros_workspace).expanduser()
            candidates.extend(
                [
                    ws / "src" / "waregv_mapping" / "maps",
                    ws / "install" / "waregv_mapping" / "share" / "waregv_mapping" / "maps",
                ]
            )

        result: List[pathlib.Path] = []
        seen = set()
        for path in candidates:
            path = path.resolve()
            key = str(path)
            if key not in seen:
                seen.add(key)
                result.append(path)
        return result

    def _directory_contains_map_data(self, directory: pathlib.Path) -> bool:
        if not directory.is_dir():
            return False
        try:
            for item in directory.iterdir():
                if item.is_file() and item.suffix.lower() in {
                    ".yaml", ".yml", ".pgm", ".png",
                }:
                    return True
                if item.is_dir():
                    try:
                        if any(
                            child.is_file()
                            and child.suffix.lower() in {".yaml", ".yml", ".pgm", ".png"}
                            for child in item.iterdir()
                        ):
                            return True
                    except OSError:
                        pass
        except OSError:
            return False
        return False

    def _resolve_map_directory(self) -> pathlib.Path:
        candidates = self._candidate_map_directories()
        for candidate in candidates:
            if self._directory_contains_map_data(candidate):
                candidate.mkdir(parents=True, exist_ok=True)
                return candidate
        fallback = (
            pathlib.Path.home()
            / "waregv" / "waregv_ws" / "src" / "waregv_mapping" / "maps"
        )
        fallback.mkdir(parents=True, exist_ok=True)
        return fallback

    # --- name / path helpers ---------------------------------------------

    @staticmethod
    def _safe_map_name(name: str) -> str:
        return (
            re.sub(r"[^A-Za-z0-9_.-]+", "_", (name or "").strip())
            .strip("._")
        )

    def _map_paths(
        self, name: str,
    ) -> Optional[Tuple[pathlib.Path, pathlib.Path]]:
        safe_name = self._safe_map_name(name)
        if not safe_name:
            return None

        roots = [self.map_directory]
        if self.map_directory.name == safe_name:
            roots.append(self.map_directory.parent)

        checked = set()
        for root in roots:
            root = root.resolve()
            key = str(root)
            if key in checked:
                continue
            checked.add(key)

            for yaml_path in (
                root / f"{safe_name}.yaml",
                root / f"{safe_name}.yml",
            ):
                if yaml_path.exists():
                    image_path = self._find_image_near(yaml_path.parent, safe_name)
                    if image_path is not None:
                        return yaml_path, image_path

            nested_dir = root / safe_name
            for yaml_path in (
                nested_dir / f"{safe_name}.yaml",
                nested_dir / f"{safe_name}.yml",
            ):
                if yaml_path.exists():
                    image_path = self._find_image_near(nested_dir, safe_name)
                    if image_path is not None:
                        return yaml_path, image_path
        return None

    @staticmethod
    def _find_image_near(
        directory: pathlib.Path, safe_name: str,
    ) -> Optional[pathlib.Path]:
        for ext in (".pgm", ".png", ".jpeg", ".jpg"):
            path = directory / f"{safe_name}{ext}"
            if path.exists() and path.is_file():
                return path
        return None

    def _find_yaml_only(self, name: str) -> Optional[pathlib.Path]:
        safe_name = self._safe_map_name(name)
        if not safe_name:
            return None

        roots = [self.map_directory, self.map_directory.parent]
        checked = set()
        for root in roots:
            root = root.resolve()
            if str(root) in checked:
                continue
            checked.add(str(root))
            for candidate in (
                root / f"{safe_name}.yaml",
                root / f"{safe_name}.yml",
                root / safe_name / f"{safe_name}.yaml",
                root / safe_name / f"{safe_name}.yml",
            ):
                if candidate.exists() and candidate.is_file():
                    return candidate
        return None

    # --- listing / info --------------------------------------------------

    def list_map_names(self) -> List[str]:
        names = set()
        root = self.map_directory
        if not root.exists():
            return []
        try:
            for ext in ("*.yaml", "*.yml"):
                for yaml_path in root.rglob(ext):
                    if yaml_path.is_file():
                        names.add(yaml_path.stem)
        except OSError as exc:
            self.get_logger().warning(f"Could not scan map directory: {exc}")
        return sorted(names)

    def get_map_info(self, name: str) -> Dict[str, Any]:
        safe_name = self._safe_map_name(name)
        if not safe_name:
            return {
                "ok": False, "name": name, "exists": False,
                "detail": "Map name is empty.",
            }
        yaml_path = self._find_yaml_only(safe_name)
        if yaml_path is None:
            return {
                "ok": False, "name": safe_name, "exists": False,
                "yaml": None, "image": None,
                "detail": (
                    f"YAML file for map '{safe_name}' was not found. "
                    f"Searched under '{self.map_directory}'."
                ),
            }
        image_path = self._find_image_near(yaml_path.parent, safe_name)
        return {
            "ok": True,
            "name": safe_name,
            "exists": True,
            "yaml": str(yaml_path),
            "image": str(image_path) if image_path else None,
            "image_available": image_path is not None,
        }

    # --- archive ---------------------------------------------------------

    def map_archive(self, name: str):
        safe_name = self._safe_map_name(name)
        if not safe_name:
            raise FileNotFoundError("Map name is empty.")

        paths = self._map_paths(safe_name)
        if paths is None:
            info = self.get_map_info(safe_name)
            raise FileNotFoundError(
                info.get("detail", f"Map '{safe_name}' is not available.")
            )

        yaml_path, image_path = paths
        archive = io.BytesIO()
        with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as zip_file:
            zip_file.writestr(yaml_path.name, yaml_path.read_bytes())
            zip_file.writestr(image_path.name, image_path.read_bytes())
            for extension in (".posegraph", ".data"):
                candidate = yaml_path.parent / (safe_name + extension)
                if candidate.exists() and candidate.is_file():
                    zip_file.writestr(candidate.name, candidate.read_bytes())
        archive.seek(0)
        return safe_name, archive

    # --- disk save -------------------------------------------------------

    def map_exists(self, name: str) -> bool:
        safe_name = self._safe_map_name(name)
        if not safe_name:
            return False
        nested = self.map_directory / safe_name
        if nested.is_dir() and any(
            (nested / f"{safe_name}{ext}").exists()
            for ext in (".yaml", ".yml")
        ):
            return True
        return any(
            (self.map_directory / f"{safe_name}{ext}").exists()
            for ext in (".yaml", ".yml")
        )

    def save_map_to_disk(
        self,
        name: str,
        pgm_bytes: bytes,
        yaml_bytes: Optional[bytes] = None,
        overwrite: bool = False,
    ) -> Dict[str, Any]:
        safe_name = self._safe_map_name(name)
        if not safe_name:
            raise ValueError(
                "Map name is empty or contains only invalid characters."
            )
        if self.map_exists(safe_name) and not overwrite:
            raise FileExistsError(
                f"Map '{safe_name}' already exists at "
                f"'{self.map_directory}'. Choose another name or enable overwrite."
            )
        target_dir = self.map_directory / safe_name
        target_dir.mkdir(parents=True, exist_ok=True)
        pgm_path = target_dir / f"{safe_name}.pgm"
        yaml_path = target_dir / f"{safe_name}.yaml"
        pgm_path.write_bytes(pgm_bytes)
        if yaml_bytes is None:
            yaml_text = (
                f"image: {safe_name}.pgm\n"
                f"resolution: 0.050000\n"
                f"origin: [0.000000, 0.000000, 0.000000]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.196\n"
            )
            yaml_path.write_text(yaml_text, encoding="utf-8")
        else:
            yaml_path.write_bytes(yaml_bytes)
        self.get_logger().info(f"Saved map '{safe_name}' to {target_dir}")
        return {
            "ok": True,
            "name": safe_name,
            "directory": str(target_dir),
            "pgm": str(pgm_path),
            "yaml": str(yaml_path),
        }


import os  # noqa: E402  (kept at end to match original import style)