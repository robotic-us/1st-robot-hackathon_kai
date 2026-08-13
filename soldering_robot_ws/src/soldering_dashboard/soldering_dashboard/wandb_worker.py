"""Read-only W&B polling worker; API calls never block the GUI thread."""

from __future__ import annotations

import threading

from PyQt5 import QtCore

from .dashboard_core import normalized_wandb_history


class WandbWorker(QtCore.QThread):
    run_update = QtCore.pyqtSignal(object)
    worker_status = QtCore.pyqtSignal(str)

    def __init__(self, parent=None, poll_s: float = 5.0):
        super().__init__(parent)
        self._poll_s = max(2.0, float(poll_s))
        self._entity = ""
        self._project = ""
        self._stop_event = threading.Event()
        self._wake_event = threading.Event()
        self._target_lock = threading.Lock()

    def set_target(self, entity: str, project: str) -> None:
        with self._target_lock:
            self._entity = entity.strip()
            self._project = project.strip()
        self._wake_event.set()

    def refresh(self) -> None:
        self._wake_event.set()

    def stop(self) -> None:
        self._stop_event.set()
        self._wake_event.set()

    def _target(self) -> tuple[str, str]:
        with self._target_lock:
            return self._entity, self._project

    def run(self) -> None:
        try:
            import wandb
        except ImportError:
            self.worker_status.emit("missing: wandb")
            return
        api = None
        while not self._stop_event.is_set():
            entity, project = self._target()
            if not entity or not project:
                self.worker_status.emit("waiting for entity/project")
            else:
                try:
                    if api is None:
                        api = wandb.Api(timeout=5)
                    runs = api.runs(
                        f"{entity}/{project}",
                        order="-created_at",
                        per_page=1,
                    )
                    run = next(iter(runs), None)
                    if run is None:
                        self.worker_status.emit("connected; no runs")
                        self.run_update.emit(
                            {
                                "entity": entity,
                                "project": project,
                                "state": "no runs",
                                "history": normalized_wandb_history([]),
                            }
                        )
                    else:
                        keys = [
                            "epoch",
                            "train/loss",
                            "validation/loss",
                            "validation/accuracy",
                            "learning_rate",
                        ]
                        rows = run.history(
                            samples=500, keys=keys, pandas=False
                        )
                        summary = dict(run.summary)
                        system_metrics = dict(run.system_metrics or {})
                        self.run_update.emit(
                            {
                                "entity": entity,
                                "project": project,
                                "id": run.id,
                                "name": run.name or run.id,
                                "state": run.state,
                                "url": run.url,
                                "created_at": str(run.created_at),
                                "config": dict(run.config),
                                "summary": summary,
                                "system_metrics": system_metrics,
                                "history": normalized_wandb_history(rows),
                            }
                        )
                        self.worker_status.emit(f"connected: {run.state}")
                except ValueError as exc:
                    api = None
                    if "Could not find project" in str(exc):
                        self.worker_status.emit(
                            "connected; project has no training run yet"
                        )
                        self.run_update.emit(
                            {
                                "entity": entity,
                                "project": project,
                                "state": "not created",
                                "history": normalized_wandb_history([]),
                            }
                        )
                    else:
                        self.worker_status.emit(f"error: ValueError: {exc}")
                except Exception as exc:  # noqa: BLE001 - report API failures
                    api = None
                    self.worker_status.emit(
                        f"error: {type(exc).__name__}: {exc}"
                    )
            self._wake_event.wait(self._poll_s)
            self._wake_event.clear()
