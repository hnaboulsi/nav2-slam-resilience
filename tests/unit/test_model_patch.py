from pathlib import Path

import pytest

from nav2_slam_resilience.model_patch import ModelPatchError, patch_lidar_noise_stddev

FIXTURE = Path(__file__).parent / "fixtures" / "turtlebot3_burger_model.sdf"


@pytest.fixture
def real_vendor_sdf() -> str:
    """The actual model.sdf shipped by ros-jazzy-turtlebot3-gazebo, unmodified.

    Extracted directly from the built Docker image
    (/opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf)
    so this test exercises the patch against the real vendor file, not a
    hand-written approximation of it.
    """
    return FIXTURE.read_text()


class TestPatchLidarNoiseStddev:
    def test_replaces_only_the_lidar_stddev(self, real_vendor_sdf):
        patched = patch_lidar_noise_stddev(real_vendor_sdf, 0.05)
        assert "<stddev>0.05</stddev>" in patched
        # The stock lidar value must be gone...
        assert "<stddev>0.01</stddev>" not in patched
        # ...but the IMU's unrelated noise stddevs must be untouched.
        assert "<stddev>2e-4</stddev>" in patched
        assert "<stddev>1.7e-2</stddev>" in patched

    def test_only_one_stddev_changes(self, real_vendor_sdf):
        before = real_vendor_sdf.count("<stddev>")
        patched = patch_lidar_noise_stddev(real_vendor_sdf, 0.05)
        after = patched.count("<stddev>")
        assert before == after  # same number of elements, just one value changed

    def test_zero_stddev_is_a_valid_baseline(self, real_vendor_sdf):
        patched = patch_lidar_noise_stddev(real_vendor_sdf, 0.0)
        assert "<stddev>0.0</stddev>" in patched

    def test_rejects_negative_stddev(self, real_vendor_sdf):
        with pytest.raises(ModelPatchError, match="stddev must be >= 0"):
            patch_lidar_noise_stddev(real_vendor_sdf, -0.01)

    def test_sensor_not_found_raises(self):
        with pytest.raises(ModelPatchError, match="no <sensor"):
            patch_lidar_noise_stddev("<sdf></sdf>", 0.05)

    def test_missing_stddev_inside_sensor_raises(self):
        sdf = '<sensor name="hls_lfcd_lds" type="gpu_lidar"><lidar></lidar></sensor>'
        with pytest.raises(ModelPatchError, match="no <stddev>"):
            patch_lidar_noise_stddev(sdf, 0.05)

    def test_ambiguous_duplicate_sensor_blocks_raises(self):
        block = '<sensor name="hls_lfcd_lds" type="gpu_lidar"><stddev>0.01</stddev></sensor>'
        with pytest.raises(ModelPatchError, match="found 2"):
            patch_lidar_noise_stddev(block + block, 0.05)

    def test_ambiguous_duplicate_stddev_inside_sensor_raises(self):
        sdf = (
            '<sensor name="hls_lfcd_lds" type="gpu_lidar">'
            "<stddev>0.01</stddev><stddev>0.02</stddev>"
            "</sensor>"
        )
        with pytest.raises(ModelPatchError, match="found 2"):
            patch_lidar_noise_stddev(sdf, 0.05)

    def test_does_not_mutate_input_string(self, real_vendor_sdf):
        # Strings are immutable in Python, but guard the contract explicitly:
        # the function must return a new string, not rely on in-place effects.
        snapshot = real_vendor_sdf
        patch_lidar_noise_stddev(real_vendor_sdf, 0.05)
        assert real_vendor_sdf == snapshot
