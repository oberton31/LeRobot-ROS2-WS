# RRT-Connect

## RRT
Rapidly-exploring Random Trees (RRTs) is a common single-query technique for planning in high dimensional spaces where grid-based methods are computationally infeasible. The algorithm uses sampling to explore the search space, which is inherently biased towards the larger unsearched regions of the space. The high level pseudocode is as follows:

```bash
Graph G
G.add_vertex(q0)
while q_new not within goal region
    # 1. Sample new configuration (or greedily sample goal)
    q_rand = generate_new_config()
    q_near = nearest_vertex()

    # 2. Extend graph towards new vertex
    q_new = step_towards_vertex(q_rand, q_near)
    G.add_vertex(q_new)
    G.add_edge(q_near, q_new)

plan = backtrack(q_new, q0)
```

where in step_twoards_vertex(), the algorithm linearly interpolates from q_near to q_new with a max step size of k, or until it enters C_obs (are of configuration space with obstacles). The resultant paths typically consist of linear interpolations between waypoints, with post-processing often performed to shorten the paths.

### RRT-Connect
RRT-Connect is a bidirectional variant of RRT that aims to speed up the search time. It does this by building two trees: one from the initial configuration, and one from the goal configuration. The two high level trees take turn expanding, and attempt to greedily connect during each turn. The high level pseudocode is:

```bash
Graph G_init
Graph G_goal

Graph G_active = G_init
Graph G_other = G_goal

while G_init and G_goal not connected:
    q_rand = generate_new_config()
    q_near = nearest_vertex(G_active, q_rand)
    
    # 1. Standard single step from G_active toward q_rand
    if can_step(q_near, q_rand):
        q_new = step_towards_vertex(q_near, q_rand)
        G_active.add_vertex(q_new)
        G_active.add_edge(q_near, q_new)

        # 2. Greedy connection: G_other repeatedly steps toward q_new
        q_target = q_new
        q_near_other = nearest_vertex(G_other, q_target)
        
        while can_step(q_near_other, q_target):
            q_new_other = step_towards_vertex(q_near_other, q_target)
            G_other.add_vertex(q_new_other)
            G_other.add_edge(q_near_other, q_new_other)
            q_near_other = q_new_other
            
            if q_new_other == q_target:
                # Trees have met; connection successful
                mark_connected(G_init, G_goal)
                break

    # 3. Swap active tree roles for the next iteration
    swap(G_active, G_other)
```