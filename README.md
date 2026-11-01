# Emergent Control of Ant Learning Walks in a Mobile Robot

This repository contains the robotic implementation of an insect-inspired model of desert ant **learning walks (LWs)**. Before starting to forage, naive ants explore the surroundings of their nest through short, spiralling walks, interspersed with **pirouettes** during which they scan and learn the visual panorama. Successive walks become longer and straighter as the ants acquire visual memories.

The model proposes that a single error signal—the **angular mismatch between the nest direction estimated by path integration (PI) and the direction predicted by a visual compass (VC)**—controls the balance between exploration and exploitation. Large errors promote slow movement, frequent scans and early returns to the nest. As visual memories accumulate, the mismatch decreases, allowing longer, straighter walks and, eventually, foraging-like behaviour.

Figure 1 illustrates this error signal: PI and the visual compass provide separate predictions of the nest direction, and their angular disagreement regulates learning-walk behaviour.

<img src="media/Epiv.jpg" alt="Diagram of the angular mismatch between the visual and path-integration predictions" width="500">

*Figure 1. Angular mismatch between the nest directions predicted by path integration and the visual compass.*

The aim of this robotic implementation is to test whether the same mechanism survives the **reality gap**: real sensors, uneven terrain, wheel slippage, changing illumination and imperfect panoramic images. The associated manuscript is *Emergent Control of Ant Learning Walks in a Mobile Robot* (Misiek, Philippides, Graham and Knight, 2026; arXiv submission).

## Experimental results

The robot autonomously selects when to explore, perform pirouettes, acquire visual memories and return towards the nest. Across successive walks, exploration gives way to longer, straighter trajectories as the visual compass becomes more reliable. The acquired memories also support subsequent visual homing.

Figure 2 shows representative learning walks recorded during outdoor experiments, including the robot's exploratory trajectories around the nest and the experimental landmarks.

<img src="media/example_LWs.png" alt="Examples of robotic learning-walk trajectories" width="470">

*Figure 2. Examples of learning-walk trajectories recorded during outdoor experiments.*

The complete set of recorded trajectories is shown in Figure 3, with learning walks and subsequent visual homing tests presented separately.

<img src="media/all_LWs.png" alt="Recorded learning walks and zero-vector visual homing tests" width="500">

*Figure 3. Overview of the outdoor experiments: (A) learning-walk trajectories and (B) zero-vector visual homing tests.*

The experimental data (CSV files) are publicly available in this repository.

## Robotic platforms

Outdoor experiments were conducted near the Library Square at the University of Sussex. As shown in Figure 4, three white cylinders reproduce a scaled-up version of the landmark arrangement used in desert-ant learning-walk experiments.

<img src="media/robot_library.jpg" alt="Robot and landmark cylinders at the University of Sussex" width="620">

*Figure 4. Outdoor experimental arena near the Library Square, University of Sussex, with three white cylindrical landmarks.*

Both platforms are much larger than an ant, **not because the neural model requires a large robot**, but to accommodate the requirements of extended outdoor experiments: sufficient battery capacity to operate for several hours, onboard computing and sensors, and GNSS equipment for accurate trajectory recording. Large wheels also allow the robot to travel over grass and uneven terrain.

The outdoor differential-drive robot used for the learning-walk experiments (Figure 5) weighs approximately **15 kg** and has **27 cm wheels**. Its high-level navigation and visual processing run on an **NVIDIA Jetson TX1**, with an **Arduino** handling low-level sensor acquisition and **RoboClaw** motor controllers driving the wheels. A panoramic camera provides visual input; inertial heading and wheel-encoder displacement are integrated to maintain a home vector. **RTK-GNSS (GPS) provides ground-truth positions for recording and analysis; it does not supply the robot's visual homing direction.**

<img src="media/robot.jpg" alt="Differential-drive robot used in outdoor experiments" width="500">

*Figure 5. Custom-built differential-drive robot used in the outdoor learning-walk experiments, with large wheels and a panoramic camera.*

An earlier platform, shown in Figure 6, used mecanum wheels during development.

<img src="media/mecanum_robot.jpg" alt="Earlier mecanum-wheel robotic platform" width="500">

*Figure 6. Earlier robot platform with mecanum wheels, used during development.*

## Navigation architecture

The model combines three interacting components:

1. **Path integration (PI):** estimates the nestward direction from inertial heading and wheel-encoder displacement.
2. **Visual compass (VC):** uses panoramic views and mushroom-body-inspired visual memories to predict the nestward direction.
3. **Learning-walk controller:** uses the angular mismatch between PI and VC predictions to regulate exploration, pirouettes, visual learning and the transition towards foraging.

The organisation of the robot's navigation and control system is shown in Figure 7.

<img src="media/robot_diagram.png" alt="Robot navigation and control architecture" width="650">

*Figure 7. Overview of the robot's navigation and control architecture.*

The visual compass learns from images acquired by the panoramic camera. Figure 8 compares raw fisheye images with their unwrapped panoramic representations. The two views were acquired at different positions and times, illustrating changes in the relative positions of nearby landmarks and in the visual appearance of the scene.

<img src="media/panoramic_views.png" alt="Fisheye camera images and unwrapped panoramic views" width="680">

*Figure 8. Raw fisheye views (A, C) and corresponding unwrapped panoramas (B, D) acquired by the robot.*

### Model size versus robot size

The large ground robots are experimental platforms, not a requirement of the mushroom-body-inspired visual-memory architecture. Related visual-memory models have also been demonstrated with micro-aerial vehicles. For example, [Gattaux et al. (2026), *Antflie: Frugal Visual Teach and Repeat on Narrow FoV Micro-Drones*](https://doi.org/10.1109/LRA.2026.3667486), use a related mushroom-body-inspired approach with a **33 g Crazyflie 2.1-based drone**. Their neural network was evaluated using offboard computation; their navigation task and controller are distinct from the PI–VC mismatch-driven learning-walk model implemented here.

## Related research

- **Robotic implementation:** Misiek, T., Philippides, A., Graham, P. & Knight, J. (2026). *Emergent Control of Ant Learning Walks in a Mobile Robot*. arXiv submission. **Public arXiv identifier/link to be added once available.**
- **Computational model of learning walks:** [Misiek, T., Philippides, A., Graham, P. & Knight, J. (2026). *Emergent control of ant learning walks using the mismatch between path-integration and visual cues.* bioRxiv.](https://doi.org/10.64898/2026.05.29.728637)
- **Insect-inspired flying robot:** [Gattaux et al. (2026). *Antflie: Frugal Visual Teach and Repeat on Narrow FoV Micro-Drones.*](https://doi.org/10.1109/LRA.2026.3667486)
- **Long-range visual homing in a drone:** [Ou, D. et al. (2026). *Efficient robot navigation inspired by honeybee learning flights.* Nature 653, 1039–1046.](https://doi.org/10.1038/s41586-026-10461-3)

## Next step: learning flights in flying robots

The next step is to transfer this error-driven model of **active visual learning** to flying robots, to investigate the learning flights (LFs) of **bees and wasps**. As with ant learning walks, the aim is not only to demonstrate that memories acquired near the nest can later support homing, but to explain **when and where the animal acquires those memories, how its exploratory trajectory is controlled, and how learning gives way to foraging**.

Recent results show that insect-inspired visual homing can operate over much longer distances than those tested here. In [Ou et al. (2026)](https://doi.org/10.1038/s41586-026-10461-3), the *Bee-Nav* quadcopter combined path integration with a compact visual homing network, achieving homing to within **0.5 m** on **100% of 30–110 m flights** and **70% of 200–600 m flights** in the reported experiments. The visual networks occupied only **3.4 kB** or **42.3 kB** of memory and ran on a Raspberry Pi 4.

Importantly, although its learning flights were executed autonomously, *Bee-Nav* used a **predefined, algorithmically generated “wasp-like” trajectory** to collect visual memories. The authors explicitly state that they did not reproduce the varying learning-flight trajectories of real honeybees. Thus, their work demonstrates efficient acquisition and use of visual memories for homing, but **not a biologically motivated online mechanism controlling the structure and progression of learning flights**.

---

**Author:** Thomas Misiek · University of Sussex, UK
