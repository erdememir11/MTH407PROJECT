from pathlib import Path

from analysis_pipeline import run_noise_analysis


if __name__ == "__main__":
    run_noise_analysis(
        noise_mode="realistic_los_nlos",
        output_dir=Path("outputs") / "03_realistic_los_nlos_noise",
        title="Realistic LOS/NLOS Noise",
        seed_base=30_000,
    )
