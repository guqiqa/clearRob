"""
YOLO 训练回调 — 每 epoch 结束后自动记录关键指标到独立日志文件。

用法:
  from train_logger import TrainingLogger
  model.train(data=..., callbacks={"on_fit_epoch_end": TrainingLogger("v6_yolo11s")})

或作为独立回调类:
  logger = TrainingLogger(run_name="v6_yolo11s")
  model.add_callback("on_fit_epoch_end", logger)
"""

import os
import json
from datetime import datetime
from pathlib import Path


class TrainingLogger:
    """每 epoch 结束后记录指标到 JSON 日志文件"""

    def __init__(self, run_name=None, output_dir=None):
        self.run_name = run_name or f"train_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.output_dir = Path(output_dir) if output_dir else Path.cwd() / "runs" / self.run_name
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = self.output_dir / "epoch_metrics.jsonl"
        self.summary_path = self.output_dir / "training_summary.txt"
        self.history = []
        self.best_map50 = 0.0
        self.best_epoch = 0
        self.start_time = datetime.now()

    def __call__(self, trainer):
        """Callback: ultralytics 在每 epoch 结束后调用此方法"""
        epoch = trainer.epoch + 1

        metrics_dict = trainer.metrics.__dict__ if hasattr(trainer.metrics, '__dict__') else {}
        mp = float(getattr(trainer.metrics, 'results_dict', {}).get('metrics/precision(B)', 0) or 0)
        mr = float(getattr(trainer.metrics, 'results_dict', {}).get('metrics/recall(B)', 0) or 0)
        map50 = float(getattr(trainer.metrics, 'results_dict', {}).get('metrics/mAP50(B)', 0) or 0)
        map50_95 = float(getattr(trainer.metrics, 'results_dict', {}).get('metrics/mAP50-95(B)', 0) or 0)

        # Get loss values from trainer
        box_loss = float(trainer.loss_items[0]) if hasattr(trainer, 'loss_items') and len(
            trainer.loss_items) >= 3 else 0
        cls_loss = float(trainer.loss_items[1]) if hasattr(trainer, 'loss_items') and len(
            trainer.loss_items) >= 3 else 0
        dfl_loss = float(trainer.loss_items[2]) if hasattr(trainer, 'loss_items') and len(
            trainer.loss_items) >= 3 else 0

        # F1 score
        f1 = 2 * mp * mr / (mp + mr + 1e-8)

        elapsed = (datetime.now() - self.start_time).total_seconds()

        record = {
            "epoch": epoch,
            "timestamp": datetime.now().isoformat(),
            "elapsed_minutes": round(elapsed / 60, 1),
            "box_loss": round(box_loss, 4),
            "cls_loss": round(cls_loss, 4),
            "dfl_loss": round(dfl_loss, 4),
            "precision": round(mp, 4),
            "recall": round(mr, 4),
            "f1": round(f1, 4),
            "mAP50": round(map50, 4),
            "mAP50_95": round(map50_95, 4),
            "lr": round(float(trainer.optimizer.param_groups[0]['lr']), 8),
            "gpu_mem_mb": round(float(trainer.memory) if hasattr(trainer, 'memory') and trainer.memory else 0, 1),
        }

        self.history.append(record)

        # Track best
        is_best = False
        if record["mAP50"] > self.best_map50:
            self.best_map50 = record["mAP50"]
            self.best_epoch = epoch
            is_best = True

        # Write JSONL line
        with open(self.log_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")

        # Update summary file
        self._write_summary(record, is_best)

        # Console print
        best_mark = " *BEST*" if is_best else ""
        elapsed_h = elapsed / 3600
        eta_h = elapsed_h / epoch * (trainer.epochs - epoch)
        print(
            f"  [E{epoch:>3}/{trainer.epochs}] "
            f"mAP50={map50:.4f} mAP50-95={map50_95:.4f} "
            f"P={mp:.4f} R={mr:.4f} F1={f1:.4f} "
            f"box={box_loss:.3f} cls={cls_loss:.3f} "
            f"elapsed={elapsed_h:.1f}h ETA={eta_h:.1f}h{best_mark}"
        )

    def _write_summary(self, latest, is_best):
        lines = [
            f"Training: {self.run_name}",
            f"Started:  {self.start_time.isoformat()}",
            f"Updated:  {datetime.now().isoformat()}",
            f"{'─' * 55}",
            f"Epoch:        {latest['epoch']}",
            f"Elapsed:      {latest['elapsed_minutes']:.0f} min",
            f"",
            f"  mAP50:       {latest['mAP50']:.4f}",
            f"  mAP50-95:    {latest['mAP50_95']:.4f}",
            f"  Precision:   {latest['precision']:.4f}",
            f"  Recall:      {latest['recall']:.4f}",
            f"  F1:          {latest['f1']:.4f}",
            f"",
            f"  box_loss:    {latest['box_loss']:.4f}",
            f"  cls_loss:    {latest['cls_loss']:.4f}",
            f"  dfl_loss:    {latest['dfl_loss']:.4f}",
            f"  GPU mem:     {latest['gpu_mem_mb']:.0f} MB",
            f"",
            f"  Best mAP50:  {self.best_map50:.4f} @ E{self.best_epoch}",
            f"  {'*** NEW BEST ***' if is_best else ''}",
        ]
        with open(self.summary_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")

    def get_best(self):
        return {"epoch": self.best_epoch, "mAP50": self.best_map50}

    def get_history(self):
        return self.history
