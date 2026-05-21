import generate_01_environment
import run_02_baseline_constant_noise
import run_03_realistic_los_nlos_noise
from pathlib import Path


if __name__ == "__main__":
    generate_01_environment.main()
    run_02_baseline_constant_noise.run_noise_analysis(
        noise_mode="baseline_constant_los",
        output_dir=Path("outputs") / "02_baseline_constant_noise",
        title="Baseline Constant LOS Noise",
        seed_base=20_000,
    )
    run_03_realistic_los_nlos_noise.run_noise_analysis(
        noise_mode="realistic_los_nlos",
        output_dir=Path("outputs") / "03_realistic_los_nlos_noise",
        title="Realistic LOS/NLOS Noise",
        seed_base=30_000,
    )
