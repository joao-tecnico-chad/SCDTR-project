# 7 - Distributed Algorithms

> [!abstract] Summary
> Three optimization algorithms — Consensus ADMM, ADMM, and Dual Decomposition — solve the same energy-minimization LP in a distributed way. No central computer needed: each node knows only its own row of K and iterates by exchanging proposals over CAN. All three converge to the same point with equal costs.

Back to [[SCDEEC Home]]

---

## The Optimization Problem — Formal Statement

Given:
- Gain matrix **K** (3×3, measured during calibration)
- Background **o** (3×1, measured during calibration)
- References **L_ref** (3×1, set by occupancy — 10 or 30 lux)
- Cost coefficients **c** (3×1, default = [1, 1, 1])

Find duty cycles **d** (3×1) that:

$$\min_{\mathbf{d}} \; \sum_{i=1}^{3} c_i d_i$$

subject to:

$$\mathbf{K}\mathbf{d} + \mathbf{o} \geq \mathbf{L}_{ref} \quad \text{(minimum illuminance at every desk)}$$
$$\mathbf{0} \leq \mathbf{d} \leq \mathbf{1} \quad \text{(duty cycle bounds)}$$

This is a **Linear Program (LP)**.

### Why not just solve it centrally?

We could solve this LP in milliseconds on any computer. But:
1. **Requires full K matrix centrally** — Node 1 only knows its own row of K (learned during calibration). Sending K to a central server requires additional communication.
2. **Single point of failure** — if the central server fails, the whole system stops.
3. **This is a course project** — the point is to implement and compare distributed algorithms.
4. **Scalability** — in a real office building with 1000 lamps, a distributed approach scales better.

### Why is it called "distributed"?

In our implementation, each node starts with knowledge of only:
- Its own row of K (from calibration)
- Its own background o_i
- Its own reference L_ref,i
- Its own cost c_i

Through the **gain exchange** phase after calibration, every node gets the full K matrix. But even so, the optimization runs **locally on each node** — there's no coordinator that collects all d proposals and sends back the answer. Nodes exchange proposals directly via CAN and each node updates its own estimate.

---

## Common Setup: Gain Exchange

Before any algorithm can run, all nodes must have the full K matrix, all reference values, all costs, and all backgrounds. This happens in the **gain exchange** phase:

After calibration completes:
1. Each node broadcasts 6 CAN frames: K row (3 frames), baseline, L_ref, cost
2. Each node receives peers' 6 frames
3. When all rows received (or 5-second timeout), algorithm initializes

The shared data structure `ConsensusState cons` holds this shared information and is used by all three algorithms.

---

## Algorithm 1: Consensus ADMM (Our Main Algorithm)

### High-level idea

Imagine each node has its own "proposal" for what every node's duty should be. The algorithm works by:
1. Each node proposes duty values that minimize its own cost while satisfying all illuminance constraints
2. Nodes share their proposals over CAN
3. Nodes average the received proposals
4. Repeat until all nodes agree

When all nodes agree on the same duty vector, that's the solution.

### The local optimization subproblem

Each node $i$ solves:

$$\min_{d_i} \; c_i d_i + \frac{\rho}{2}(d_i - \bar{d}_i)^2 \quad \text{s.t. illuminance constraints}$$

The first term ($c_i d_i$) minimizes node i's energy cost.
The second term ($\frac{\rho}{2}(d_i - \bar{d}_i)^2$) penalizes deviating from the network average $\bar{d}_i$.

This is a simple quadratic, and the unconstrained solution is:

$$d_i^{unc} = \bar{d}_i - \frac{c_i}{\rho}$$

Then we must satisfy the illuminance constraints. For each constraint $k$ (desk $k$ must have enough light):

$$\sum_{j \neq i} K_{kj} \bar{d}_j + K_{ki} d_i + o_k \geq L_{ref,k}$$

Solving for the minimum $d_i$ required for constraint $k$:

$$d_i^{(k)} = \frac{L_{ref,k} - o_k - \sum_{j \neq i} K_{kj} \bar{d}_j}{K_{ki}}$$

The actual minimum feasible $d_i$ is:
$$d_{min} = \max_k(d_i^{(k)}, 0)$$

And the final proposal:
$$d_i = \max(d_i^{unc}, d_{min})$$

**Feasibility always beats cost savings.**

### In code (consensus.h):

```c
// Unconstrained optimum: shift down by cost/rho
float d_unc = cons.d_avg[i] - cons.c[i] / CONS_RHO;

// Find tightest constraint
float d_min = 0.0f;
for (int k = 0; k < cons.numNodes; k++) {
    float lux_k = cons.o[k];
    for (int j = 0; j < cons.numNodes; j++) {
        if (j != i) lux_k += cons.K[k][j] * cons.d_avg[j];
    }
    if (cons.K[k][i] > 0.0f) {
        float d_needed = (cons.L_ref[k] - lux_k) / cons.K[k][i];
        if (d_needed > d_min) d_min = d_needed;
    }
}

// Feasibility wins
float d_new = (d_unc > d_min) ? d_unc : d_min;
cons.d[i] = constrain(d_new, 0.0f, 1.0f);
```

### The averaging step:

After all nodes broadcast their proposals and receive each other's proposals:

$$\bar{d}_j^{(k+1)} = \frac{1}{N} \sum_{\text{all nodes}} d_j^{(\text{that node's proposal})}$$

All three nodes proposed a value for each duty variable. Averaging them drives all proposals toward consensus.

### Convergence check:

$$\max_j |\bar{d}_j^{(k+1)} - \bar{d}_j^{(k)}| < \epsilon = 10^{-3}$$

If the average doesn't change by more than 0.1% duty in any dimension, the algorithm has converged.

### Parameters:
- **CONS_RHO = 2.0:** Penalty for deviating from average. Higher rho → faster consensus but cost term has less effect.
- **CONS_MAX_ITER = 50:** Maximum iterations. At 100 ms per iteration, this is 5 seconds.
- **CONSENSUS_PERIOD_MS = 100:** One iteration per 100 ms.

### Why rho = 2.0?

At each iteration:
- Cost pushes $d_i$ down by $c_i / \rho$. With $c_i = 1$ and $\rho = 2$, that's 0.5 duty shift maximum.
- The consensus penalty pulls $d_i$ toward the average.

If rho is too small (e.g., 0.5), the cost term dominates and nodes might propose very different values, slowing convergence. If rho is too large (e.g., 100), the consensus term dominates and costs have almost no effect. rho = 2.0 is a reasonable middle ground.

---

## Algorithm 2: ADMM (Alternating Direction Method of Multipliers)

### What makes ADMM different from Consensus?

Standard ADMM explicitly separates the optimization problem into:
1. **d-update:** minimize cost (local, cheap — each node does its own)
2. **z-update:** project onto the constraint set (ensures feasibility — computed from shared d,u)
3. **u-update:** adjust the dual variable (tracks constraint violations)

This is more theoretically rigorous than Consensus ADMM — it has formal convergence guarantees under mild conditions.

### The three update steps:

**Step 1 — d-update (each node independently):**

Minimize $c_i d_i + \frac{\rho}{2}||d_i - z_i + u_i||^2$ over $d_i$:

$$d_i^{(k+1)} = z_i^{(k)} - u_i^{(k)} - \frac{c_i}{\rho}$$

Then clamp to [0,1].

This is the closed-form solution — just arithmetic, no iterative subproblem needed.

**Step 2 — z-update (feasibility projection):**

The z variable represents the "agreed-upon" duty vector projected onto the feasible set. Start from d + u, then iteratively push z toward feasibility:

For each violated constraint k (lux_k < L_ref_k):
$$z \mathrel{+}= \frac{L_{ref,k} - \text{lux}_k}{||K_k||^2} \cdot K_k$$

This is a **projection step** — the Euclidean projection of (d+u) onto the halfspace $\{z : K_k z \geq L_{ref,k} - o_k\}$.

The code does up to 20 iterations of this projection (Dykstra's algorithm). In practice, 2–3 suffice for our well-conditioned problem.

**Step 3 — u-update (dual ascent):**

$$u_i^{(k+1)} = u_i^{(k)} + d_i^{(k+1)} - z_i^{(k+1)}$$

The dual variable u accumulates the discrepancy between d and z. When d = z (consensus reached), u stops changing and the algorithm has converged.

### Convergence criterion:

$$\sqrt{\sum_i (d_i - z_i)^2} < 10^{-3}$$

(primal residual less than 0.1% duty)

### Parameters:
- **ADMM_RHO = 1.0:** Augmented Lagrangian penalty
- Same CONS_MAX_ITER = 50, CONSENSUS_PERIOD_MS = 100

### Difference from Consensus in practice:

ADMM maintains explicit dual variables (u) and a separate consensus variable (z). This makes it more sensitive to initialization and can exhibit oscillations if the algorithm switches midway through. Our experience: ADMM sometimes showed instability during rapid algorithm switching, but was stable when used from the beginning.

---

## Algorithm 3: Dual Decomposition

### What is Lagrangian relaxation?

We take the illuminance constraints and move them into the objective, penalized by "shadow prices" (Lagrange multipliers λ):

$$L(d, \lambda) = \sum_i c_i d_i + \sum_k \lambda_k (L_{ref,k} - (K_k d + o_k))$$

The parameter $\lambda_k \geq 0$ is the **price** for violating constraint k. If constraint k is satisfied with slack ($K_k d + o_k > L_{ref,k}$), the λ term is negative (contributes a "bonus"). If violated, it's positive (contributes a penalty).

The dual function $g(\lambda) = \min_d L(d,\lambda)$ is the minimum cost achievable for a given set of prices. The dual problem is to maximize $g(\lambda)$ over $\lambda \geq 0$ — find the prices that force the primal variables to satisfy constraints.

### Primal update (gradient descent on d):

The gradient of L with respect to $d_i$:
$$\frac{\partial L}{\partial d_i} = c_i - \sum_k \lambda_k K_{ki}$$

- First term ($c_i$): energy cost pushes duty down
- Second term ($\lambda_k K_{ki}$): shadow prices push duty up if we're contributing valuable light

Gradient descent step:
$$d_i^{(k+1)} = d_i^{(k)} - \alpha \left(c_i - \sum_k \lambda_k K_{ki}\right)$$

Clamp to [0,1].

### Dual update (subgradient ascent on λ):

For each constraint k, update the shadow price based on the constraint violation:
$$\lambda_k^{(k+1)} = \max\left(0, \lambda_k^{(k)} + \alpha \cdot (L_{ref,k} - (K_k d^{(k+1)} + o_k))\right)$$

- If desk k is too dark: $L_{ref,k} - \text{lux}_k > 0$ → λ increases (higher price for not providing light)
- If desk k has enough light: violation ≤ 0 → λ decreases (but clamped at 0)

### Step size decay:

$$\alpha^{(k+1)} = \alpha^{(k)} \times 0.995$$

Starting from $\alpha_0 = 0.05$, after 50 iterations: $\alpha^{(50)} = 0.05 \times 0.995^{50} \approx 0.039$.

The decay ensures convergence: large early steps explore the space, small late steps settle precisely. Without decay, the algorithm oscillates around the optimum indefinitely.

### The key trade-off:

- **Large α:** Fast initial progress but oscillates near the optimum
- **Small α:** Very slow convergence, especially far from optimum
- **Decaying α:** Best of both worlds, but sensitive to the initial value and decay rate

In practice, this makes Dual Decomposition the hardest to tune. Our choice (α₀ = 0.05, decay = 0.995) worked for our test cases but might need adjustment for different box geometries.

---

## Algorithm 4: PI Only (No Optimization)

This is the baseline — just the PI controller tracking its own reference (L_ref_i), with no coordination between nodes. Each node ignores what others are doing. The duty set by the distributed algorithms is applied as a **new reference** to the PI controller:

```c
// After consensus iteration: update PI reference to match optimizer output
float estLux = cons.o[cons.myIndex];
for (int j = 0; j < cons.numNodes; j++)
    estLux += cons.K[cons.myIndex][j] * cons.d_avg[j];
refLux = estLux;  // PI controller will track this
```

In "PI Only" mode (algorithm = 0), the reference stays fixed at whatever the user set it to.

---

## Why Equal Costs Give the Same Answer as PI Only

With c = [1, 1, 1] and reference = 20 lux at all nodes:

The unconstrained optimum for Consensus:
$$d_i^{unc} = \bar{d}_i - c_i/\rho = \bar{d}_i - 1/2 = \bar{d}_i - 0.5$$

Since all nodes have the same c and the same reference, the optimization is symmetric. The constraint-driven minimum ($d_{min}$) dominates and all algorithms find the same minimum-energy point that satisfies all constraints.

This point is exactly what a well-tuned PI controller reaches anyway (since PI converges to steady state = exactly meeting the reference with minimum energy). So "PI Only" and "Consensus" give the same result with equal costs, equal references, and no disturbances.

**This is a validation test:** if all algorithms agree, our implementations are correct.

---

## Why Unequal Costs Don't Change Much

With c = [1, 2, 3] (Node 1 cheapest, Node 3 most expensive):

The optimizer wants Node 1 to "cover" some of Node 3's light requirement, so Node 3 can dim down. But how much can Node 1 help Node 3?

**Node 1's contribution to Node 3:** K[3][1] = 1.31 lux/duty. Even at 100% duty, Node 1 adds only 1.31 lux to Node 3. Node 3 needs 20 lux. Its self-gain K[3][3] = 72.23. So:

$$d_3^{minimum} = \frac{20 - 0.03 - 1.31 \times 1.0 - 9.09 \times d_2}{72.23}$$

With d₂ ≈ 0.17 (from equal-costs case), d₂'s contribution is 9.09 × 0.17 = 1.55 lux.

$$d_3^{minimum} = \frac{20 - 0.03 - 1.31 - 1.55}{72.23} = \frac{17.11}{72.23} = 0.237$$

Vs. equal-costs case where d₃ ≈ 0.267. The difference is only 0.030 (3%). As expected, the cross-coupling is too weak to significantly redistribute load.

**Measured data (alg_comparison_unequal.csv):**

| Node | Cost | Duty (unequal) | Duty (equal) | Difference |
|------|------|---------------|--------------|-----------|
| 1 | 1 | 0.530 | 0.520 | +0.010 |
| 2 | 2 | 0.168 | 0.168 | 0.000 |
| 3 | 3 | 0.270 | 0.268 | +0.002 |

Changes are 0–1%: essentially negligible. This is a **physical limitation of the box**, not an algorithm bug.

---

## Comparison of the Three Algorithms

| Property | Consensus ADMM | Standard ADMM | Dual Decomp |
|----------|---------------|---------------|-------------|
| **Convergence speed** | 10–20 iterations | 10–30 iterations | 20–50 iterations |
| **Robustness** | Excellent | Good | Moderate |
| **Tuning** | One parameter (ρ) | One parameter (ρ) | Two parameters (α₀, decay) |
| **Theoretical guarantee** | Yes (convex LP) | Yes (formal) | Yes (but step-size dependent) |
| **Our experience** | Always stable | Sometimes jittery | Needed careful tuning |
| **Algorithm period** | 100 ms | 100 ms | 100 ms |
| **CAN frames per iteration** | 1 per node | 1 per node | 1 per node |

---

## Common Exam Questions

> [!question] "Explain Consensus ADMM in one paragraph."
> Each node maintains a proposal for what all nodes' duty cycles should be. In each iteration: (1) the node computes its proposed duty for itself by finding the cheapest duty that satisfies all illuminance constraints while staying close to the network average; (2) it broadcasts this proposal over CAN; (3) it receives proposals from other nodes; (4) it averages all proposals. The network average converges to the global optimum as the iterations proceed.

> [!question] "What is ρ (rho) in ADMM/Consensus and why is it 2.0?"
> Rho is the penalty weight for deviating from the network consensus. It balances cost minimization (which wants each node to reduce duty) against consensus (which wants all nodes to agree on the same duty vector). Too small: the cost term dominates, nodes diverge slowly. Too large: consensus is fast but the cost term is negligible. ρ = 2.0 was found empirically to give fast convergence while still allowing meaningful cost differentiation.

> [!question] "How does Dual Decomposition work?"
> It uses Lagrange multipliers (shadow prices λ_k) to represent the "cost" of violating each illuminance constraint. In each iteration: (1) each node updates its duty by gradient descent — pushing toward lower energy cost, corrected by the shadow prices for the light it provides; (2) all nodes update the shadow prices based on how much each constraint is violated — higher price if the desk is too dark. The step size decays geometrically to ensure convergence.

> [!question] "Why do all algorithms give the same answer with equal costs?"
> With equal costs c = [1,1,1], the cost term $c_i/ρ$ is identical for every node. The unconstrained optima are all the same function of the average. The constraint-driven minimum dominates, and there is a unique minimum-energy feasible point for the given references. All algorithms find this same point, which validates their correctness.

> [!question] "Why doesn't the optimizer significantly change duty cycles when costs are unequal?"
> The cross-coupling in our box is weak: K[3][1] = 1.31 (Node 1 contributes only 1.31 lux/duty to Node 3's sensor). Even at 100% duty, Node 1 can contribute only 1.31 lux to Node 3, which needs 20 lux. Node 3 must still run at ≈27% duty for its own illuminance requirement. The optimizer's ability to redistribute load is physically limited by the box geometry, not by algorithm quality.

> [!question] "How does the algorithm output become a reference for the PI controller?"
> After each consensus iteration, the algorithm estimates what the illuminance at this node would be given the consensus duty vector: estLux = o_i + Σ_j K_ij * d_avg_j. This estimated lux is set as the PI controller's reference (refLux = estLux). The PI then drives the LED to achieve this lux, tracking the optimizer's recommendation.

---

Back to [[SCDEEC Home]]
