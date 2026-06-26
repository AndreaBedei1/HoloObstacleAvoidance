from pathlib import Path
import unittest


class InterfaceFilesTest(unittest.TestCase):
    def test_message_files_exist(self):
        package_dir = Path(__file__).resolve().parents[1]

        self.assertTrue((package_dir / "msg" / "Obstacle2D.msg").exists())
        self.assertTrue((package_dir / "msg" / "Obstacle2DArray.msg").exists())
        self.assertTrue((package_dir / "msg" / "AvoidanceDebug.msg").exists())

    def test_obstacle_message_contains_required_fields(self):
        package_dir = Path(__file__).resolve().parents[1]
        text = (package_dir / "msg" / "Obstacle2D.msg").read_text(encoding="utf-8")

        for field in (
            "std_msgs/Header header",
            "string class_name",
            "float32 confidence",
            "float32 center_x",
            "float32 center_y",
            "float32 width",
            "float32 height",
            "float32 bearing_rad",
            "float32 apparent_area",
            "float32 risk",
            "bool is_tracking_valid",
        ):
            self.assertIn(field, text)


if __name__ == "__main__":
    unittest.main()

