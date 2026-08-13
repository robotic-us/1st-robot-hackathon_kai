import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


def test_dashboard_modules_import():
    from soldering_dashboard.main_window import DashboardWindow
    from soldering_dashboard.ros_worker import RosWorker
    from soldering_dashboard.wandb_worker import WandbWorker

    assert DashboardWindow is not None
    assert RosWorker is not None
    assert WandbWorker is not None
