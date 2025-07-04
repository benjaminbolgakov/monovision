
def basic_start():
    particles = [Particle(np.array([0.0, 0.0, 0.0])) for _ in range(NUM_PARTICLES)]
    true_pose = np.array([0.0, 0.0, 0.0])
    traj = [true_pose.copy()]

    for t in range(20):
        control = [1.0, np.deg2rad(10)]
        true_pose = motion_model(true_pose, control)
        traj.append(true_pose.copy())
        measurements = [sense(true_pose, lm) for lm in LANDMARKS]

        for p in particles:
            p.motion_update(control)
            p.sensor_update(measurements)

        particles = resample(particles)

        # Visualization
        plt.clf()
        plt.title("FastSLAM - Step {}".format(t))
        for lm in LANDMARKS:
            plt.plot(lm[0], lm[1], 'gx', markersize=10)
        for p in particles:
            plt.plot(p.pose[0], p.pose[1], 'r.', alpha=0.3)
        pose_est = get_best_estimate(particles)
        plt.plot(pose_est[0], pose_est[1], 'bo')
        plt.plot([p[0] for p in traj], [p[1] for p in traj], 'k--')
        plt.axis("equal")
        plt.pause(0.2)
    plt.show()
