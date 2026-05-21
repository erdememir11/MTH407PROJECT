from pathlib import Path

from analysis_pipeline import run_noise_analysis


if __name__ == "__main__":
    run_noise_analysis(
        noise_mode="baseline_constant_los",
        output_dir=Path("outputs") / "02_baseline_constant_noise",
        title="Baseline Constant LOS Noise",
        seed_base=20_000,
    )
