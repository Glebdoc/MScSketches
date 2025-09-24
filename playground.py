import numpy as np
import matplotlib.pyplot as plt

def synthetic_twist_curve(r, beta_root=30, beta_tip=12,
                          bump_pos=0.2, bump_height=6, noise=False,
                          seed=None):
    """
    Generate a synthetic blade twist curve β(r/R) with typical shape.

    Parameters
    ----------
    r : array-like
        Radial positions, normalized 0..1.
    beta_root : float
        Pitch at root (deg).
    beta_tip : float
        Pitch at tip (deg).
    bump_pos : float
        Position (r/R) of local bump near root.
    bump_height : float
        Height of that bump (deg).
    noise : float
        Random jitter amplitude (deg).
    seed : int
        Random seed for reproducibility.

    Returns
    -------
    beta : ndarray
        β distribution, deg.
    """
    rng = np.random.default_rng(seed)
    r = np.asarray(r)

    # base linear decay
    beta = beta_root + (beta_tip - beta_root) * r

    # add a Gaussian-like bump near the root
    beta += bump_height * np.exp(-((r - bump_pos) / 0.1)**2)

    # small random noise
    if noise > 0:
        beta += rng.normal(scale=noise, size=r.shape)

    return beta

# --- Example ---
r = np.linspace(0, 1, 20)
beta3 = synthetic_twist_curve(r, beta_root=28, beta_tip=14,
                              bump_height=8, seed=3)

plt.figure(figsize=(4,3))
plt.plot(r, beta3, 'd-', label="case 3")
plt.xlabel(r"$r/R$")
plt.ylabel(r"$\beta$ (deg)")
plt.ylim(0, 35)
plt.legend()
plt.grid(True, alpha=0.3)
plt.show()