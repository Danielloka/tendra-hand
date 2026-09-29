# Embodied AI research hub

Part of the project's ultimate goal (a robot that does what humans do, see `docs/roadmap.md`). This hub works toward the first big milestone on that path: an **AI system that controls two arms with two Tendra hands, mounted on a pole**, able to pick things up and, step by step, do more and more of what human hands can do.

This folder is the research base for that goal. It is a living set of notes: add to it, cross things out, and log decisions in `../log.md`.

| File | What it is |
|---|---|
| [vision.md](vision.md) | The system we want to build: body (pole, arms, hands, sensors) and brain (the layered AI architecture) |
| [roadmap.md](roadmap.md) | Stages from "learn the stack" to "bimanual, language-driven tasks", each with a *done when* |
| [math.md](math.md) | The maths and theory, in learning order, with where each topic is used in this project |
| [resources.md](resources.md) | Books, courses, papers, open-source projects, datasets, simulators, hardware |
| [ideas.md](ideas.md) | Creative bets and open questions: things that could give Tendra an edge |

## The one-paragraph version

Nobody has "solved" human-level hands, but the recipe the best labs use (2023–2026) is clear enough to follow on a small budget:
**(1)** a good body with cameras, **(2)** a way for a human to control it naturally (teleoperation) so we can record lots of demonstrations, **(3)** imitation learning on those demonstrations (ACT, Diffusion Policy, then fine-tuning an open Vision-Language-Action model like π0 or SmolVLA), **(4)** reinforcement learning in simulation for the dexterous finger skills that humans can't easily demonstrate, and **(5)** a language model on top that splits tasks into steps. The real bottleneck is **data**, not algorithms, so most of our creativity should go into collecting good data cheaply.
