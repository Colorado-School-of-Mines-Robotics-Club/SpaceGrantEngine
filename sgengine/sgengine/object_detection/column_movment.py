import time

from openVO.oakd import OAK_Camera


def main():
    """Test main function"""
    # stopped = False

    # def target():
    #     while not stopped:
    #         # when on_demand flag is set to false, computed everytime new data is ready
    #         # when on_demand flag is set to true, computed only when the function is called
    #         cam.compute_im3d()
    #         cam.compute_point_cloud()
    #         time.sleep(1)

    cam = OAK_Camera(
        display_depth=True,
        display_mono=False,
        display_rectified=False,
        display_point_cloud=False,
        compute_im3d_on_demand=False,
        compute_point_cloud_on_demand=False,
    )

    cam.start_display()
    cam.start()
    i = 0
    while i < 1000:
        single_depth_frame = cam.depth

        # Trims the top and the bottom of the depth image by 1/4 each
        single_depth_frame = cam.depth[
            int(len(cam.depth) / 4) : 3 * int(len(cam.depth) / 4)
        ]

        # Trims the left and the right of the depth image by 0.15 each
        single_depth_frame = single_depth_frame[
            :,
            int(len(single_depth_frame[0]) * 0.15) : len(single_depth_frame[0])
            - int(len(single_depth_frame[0]) * 0.15),
        ]

        # Returns the direction of the depth frame
        print(calculate_direction(single_depth_frame))
        time.sleep(0.01)
        i += 1

    input("Press Enter to continue...")

    # stopped = True
    # thread.join()
    cam.stop()


def calculate_direction(array):
    """Takes in an 2d np array and tells to move left (-1), right (1), or center (0)"""
    # First partition the array into 3 sections by columns
    array_partition = [0] * 3
    sum_partition = [None] * 3

    # Transpose the array to get the columns
    array_partition[0] = array.T[0 : int(len(array) / 3)]
    array_partition[1] = array.T[int(len(array) / 3) : 2 * int(len(array) / 3)]
    array_partition[2] = array.T[2 * int(len(array) / 3) : len(array)]

    # Sum each of the partitions
    for i in range(len(array_partition)):
        sum_partition[i] = sum_section(array_partition[i])

    # Returns which partition is the greatest depth wise
    if sum_partition[0] > sum_partition[1] and sum_partition[0] > sum_partition[2]:
        return -1

    if sum_partition[2] > sum_partition[0] and sum_partition[2] > sum_partition[1]:
        return 1

    return 0


def sum_section(section):
    """Sums all the numbers in the section"""
    section_sum = 0

    for i in range(len(section)):
        for j in range(len(section[i])):
            section_sum += section[i][j]

    return section_sum


if __name__ == "__main__":
    main()
