"""Make the three plots from results/run.npz (written by run.py)."""
import matplotlib

matplotlib.use("Agg")  # no display needed (CI, headless)
import matplotlib.pyplot as plt
import numpy as np

from model import ROOT

OUT = ROOT / "results"
NAMES = {"flat_out": "(a) flat-out", "rule_based": "(b) rule-based", "dynamic": "(c) dynamic programming"}


def main() -> None:
    d = np.load(OUT / "run.npz")
    s = d["s"]

    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(s, d["v_real"] * 3.6, "k", lw=1.2, label="real lap (2025 Monza Q, car #1)")
    ax.plot(s, d["v_val"] * 3.6, "tab:red", lw=1.0, label="model, 2025-spec power unit")
    ax.set(xlabel="distance [m]", ylabel="speed [km/h]", title="Speed trace: model vs real")
    ax.legend(), ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUT / "speed_model_vs_real.png", dpi=120)

    fig, ax = plt.subplots(figsize=(11, 4))
    for key, label in NAMES.items():
        line, = ax.plot(s, d[f"{key}_e"] / 1e6, lw=1.2, label=label)
        clip = d[f"{key}_clip"]
        ax.plot(s[clip], d[f"{key}_e"][clip] / 1e6, ".", ms=3, color=line.get_color())
    ax.axhline(d["flat_out_e"][0] / 1e6, color="grey", ls=":", lw=1, label="start SoC (end target)")
    ax.set(xlabel="distance [m]", ylabel="battery energy [MJ]",
           title="State of charge per strategy (dots = clipping: wanted to deploy, battery empty)")
    ax.legend(), ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUT / "soc_traces.png", dpi=120)

    fig, axes = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
    for ax, (key, label) in zip(axes, NAMES.items()):
        pk = d[f"{key}_pk"] / 1e3
        ax.fill_between(s, 0, np.maximum(pk, 0), color="tab:green", step="post", label="deploy")
        ax.fill_between(s, 0, np.minimum(pk, 0), color="tab:blue", step="post", label="harvest")
        ax.set(ylabel="MGU-K [kW]", title=label, ylim=(-380, 380))
        ax.grid(alpha=0.3)
    axes[0].legend(loc="upper right")
    axes[-1].set_xlabel("distance [m]")
    fig.tight_layout()
    fig.savefig(OUT / "deploy_harvest.png", dpi=120)
    print(f"wrote 3 plots to {OUT}")


if __name__ == "__main__":
    main()
