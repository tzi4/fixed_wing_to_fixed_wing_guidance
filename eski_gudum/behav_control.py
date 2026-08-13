def adjust_angles(pid_output_roll, pid_output_pitch, pid_output_yaw, min_roll = 4, max_roll=30, min_pitch=4, max_pitch=25, min_yaw=0, max_yaw=20, pieces=5):
    def adjust_angle(scaled_output, min_boundary, max_boundary, pieces):

        # Calculate the step size based on the number of pieces
        step_size = (max_boundary - min_boundary) / pieces

        # Generate boundaries and corresponding outputs
        boundaries = [min_boundary + i * step_size for i in range(1, pieces)]
        outputs = [min_boundary + i * step_size for i in range(pieces)]
        #print('boundaries: ', boundaries)
        #print('outputs: ', outputs)
        # Determine the output based on the range
        if abs(scaled_output) < min_boundary:
            return 0
        for i in range(len(boundaries)):
            if abs(scaled_output) <= boundaries[i]:
                return int(outputs[i] * (1 if scaled_output >= 0 else -1))

        # For any value exceeding the last boundary, return the maximum scaled by sign
        return max_boundary * (1 if scaled_output >= 0 else -1)

    # Adjust each angle using the same utility function with different parameters
    roll = adjust_angle(pid_output_roll, min_boundary=min_roll, max_boundary=max_roll, pieces=pieces)
    pitch = adjust_angle(pid_output_pitch, min_boundary=min_pitch, max_boundary=max_pitch, pieces=pieces)
    yaw = adjust_angle(pid_output_yaw, min_boundary=0, max_boundary=max_yaw, pieces=pieces)

    return roll, pitch, yaw


if __name__ == "__main__":
    # TESTING
    pid_output_roll = 1
    pid_output_pitch = 10
    pid_output_yaw = 6
    pieces = 30

    min_roll = 5
    max_roll = 30
    min_pitch = 5
    max_pitch = 15
    min_yaw = 5
    max_yaw = 30

    print(f"INPUT: Roll: {pid_output_roll}, Pitch: {pid_output_pitch}, Yaw: {pid_output_yaw}, Pieces: {pieces}")

    roll, pitch, yaw = adjust_angles(pid_output_roll, pid_output_pitch, pid_output_yaw) # PID outputs

    print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
