"""Train ConvNeXt-Tiny on session-split YOLO interaction crops."""

from __future__ import annotations

import argparse
from pathlib import Path
import time


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", required=True, help="root with train/ and val/")
    parser.add_argument("--output", required=True)
    parser.add_argument("--epochs", type=int, default=25)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--learning-rate", type=float, default=3.0e-4)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--freeze-backbone-epochs", type=int, default=3)
    parser.add_argument("--wandb-project", default="")
    parser.add_argument("--wandb-entity", default="")
    parser.add_argument("--wandb-run-name", default="")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    import torch
    from torch import nn
    from torch.utils.data import DataLoader
    from torchvision import datasets, transforms
    from torchvision.models import ConvNeXt_Tiny_Weights, convnext_tiny

    data_root = Path(args.data)
    if not (data_root / "train").is_dir() or not (data_root / "val").is_dir():
        raise SystemExit("data must contain train/ and val/ directories")
    if args.epochs < 1 or args.batch_size < 1:
        raise SystemExit("epochs and batch size must be positive")
    device_name = args.device
    if device_name.startswith("cuda") and not torch.cuda.is_available():
        device_name = "cpu"
    device = torch.device(device_name)
    train_transform = transforms.Compose(
        [
            transforms.RandomResizedCrop(224, scale=(0.82, 1.0)),
            transforms.RandomRotation(6),
            transforms.ColorJitter(0.2, 0.2, 0.15, 0.04),
            transforms.GaussianBlur(3, sigma=(0.1, 1.2)),
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225],
            ),
        ]
    )
    validation_transform = ConvNeXt_Tiny_Weights.DEFAULT.transforms()
    train_set = datasets.ImageFolder(data_root / "train", train_transform)
    validation_set = datasets.ImageFolder(
        data_root / "val", validation_transform
    )
    if train_set.classes != validation_set.classes:
        raise SystemExit("train and val class directories do not match")
    train_loader = DataLoader(
        train_set,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.workers,
        pin_memory=device.type == "cuda",
    )
    validation_loader = DataLoader(
        validation_set,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=device.type == "cuda",
    )
    model = convnext_tiny(weights=ConvNeXt_Tiny_Weights.DEFAULT)
    input_features = model.classifier[2].in_features
    model.classifier[2] = nn.Linear(input_features, len(train_set.classes))
    model = model.to(device)
    criterion = nn.CrossEntropyLoss(label_smoothing=0.05)
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=args.learning_rate, weight_decay=0.02
    )
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=args.epochs
    )
    best_accuracy = -1.0
    best_validation_loss = float("inf")
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    wandb_run = None
    wandb_module = None
    if args.wandb_project:
        try:
            import wandb
        except ImportError as exc:
            raise SystemExit(
                "wandb is required when --wandb-project is set"
            ) from exc
        wandb_module = wandb
        wandb_run = wandb.init(
            project=args.wandb_project,
            entity=args.wandb_entity or None,
            name=args.wandb_run_name or None,
            config={
                "architecture": "convnext_tiny",
                "classes": train_set.classes,
                "epochs": args.epochs,
                "batch_size": args.batch_size,
                "learning_rate": args.learning_rate,
                "freeze_backbone_epochs": args.freeze_backbone_epochs,
                "train_images": len(train_set),
                "validation_images": len(validation_set),
            },
        )
    for epoch in range(args.epochs):
        freeze = epoch < args.freeze_backbone_epochs
        for parameter in model.features.parameters():
            parameter.requires_grad = not freeze
        model.train()
        train_loss = 0.0
        for images, targets in train_loader:
            images, targets = images.to(device), targets.to(device)
            optimizer.zero_grad(set_to_none=True)
            loss = criterion(model(images), targets)
            loss.backward()
            optimizer.step()
            train_loss += loss.detach().item() * images.size(0)
        model.eval()
        correct = 0
        total = 0
        validation_loss = 0.0
        with torch.inference_mode():
            for images, targets in validation_loader:
                images, targets = images.to(device), targets.to(device)
                logits = model(images)
                validation_loss += (
                    criterion(logits, targets).item() * images.size(0)
                )
                predictions = logits.argmax(dim=1)
                correct += int((predictions == targets).sum())
                total += targets.numel()
        accuracy = correct / max(1, total)
        mean_validation_loss = validation_loss / max(1, total)
        scheduler.step()
        print(
            f"epoch={epoch + 1}/{args.epochs} "
            f"loss={train_loss / max(1, len(train_set)):.5f} "
            f"val_loss={mean_validation_loss:.5f} "
            f"val_accuracy={accuracy:.5f}",
            flush=True,
        )
        if wandb_run is not None:
            wandb_run.log(
                {
                    "epoch": epoch + 1,
                    "train/loss": train_loss / max(1, len(train_set)),
                    "validation/loss": mean_validation_loss,
                    "validation/accuracy": accuracy,
                    "learning_rate": scheduler.get_last_lr()[0],
                }
            )
        improved = accuracy > best_accuracy or (
            accuracy == best_accuracy
            and mean_validation_loss < best_validation_loss
        )
        if improved:
            best_accuracy = accuracy
            best_validation_loss = mean_validation_loss
            torch.save(
                {
                    "state_dict": model.state_dict(),
                    "classes": train_set.classes,
                    "val_accuracy": accuracy,
                    "val_loss": mean_validation_loss,
                    "saved_unix": time.time(),
                },
                output,
            )
    if wandb_run is not None and wandb_module is not None:
        artifact = wandb_module.Artifact(
            name="convnext-process-best",
            type="model",
            metadata={"best_validation_accuracy": best_accuracy},
        )
        artifact.add_file(str(output))
        wandb_run.log_artifact(artifact)
        wandb_run.finish()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
