# pylint: skip-file
# Needs to be refactory according to pylint at some point

import time

from openVO.oakd import OAK_Camera


def main():
    STOPPED = False

    def target():
        while not STOPPED:
            # when on_demand flag is set to false, computed everytime new data is ready
            # when on_demand flag is set to true, computed only when the function is called
            cam.compute_im3d()
            cam.compute_point_cloud()
            time.sleep(1)

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
        singleDepthFrame = cam.depth

        # Trims the top and the bottom of the depth image by 1/4 each
        singleDepthFrame = cam.depth[
            int(len(cam.depth) / 4) : 3 * int(len(cam.depth) / 4)
        ]

        # Trims the left and the right of the depth image by 0.15 each
        singleDepthFrame = singleDepthFrame[
            :,
            int(len(singleDepthFrame[0]) * 0.15) : len(singleDepthFrame[0])
            - int(len(singleDepthFrame[0]) * 0.15),
        ]

        # Returns the direction of the depth frame
        print(calculate_direction(singleDepthFrame))
        time.sleep(0.01)
        i += 1

    input("Press Enter to continue...")

    STOPPED = True
    # thread.join()
    cam.stop()


# takes in an 2d np array and tells to move left (-1), right (1), or center (0)
def calculate_direction(array):
    # First partition the array into 3 sections by columns
    arrayPartition = [0] * 3
    sumPartition = [None] * 3

    # Transpose the array to get the columns
    arrayPartition[0] = array.T[0 : int(len(array) / 3)]
    arrayPartition[1] = array.T[int(len(array) / 3) : 2 * int(len(array) / 3)]
    arrayPartition[2] = array.T[2 * int(len(array) / 3) : len(array)]

    # Sum each of the partitions
    for i in range(len(arrayPartition)):
        sumPartition[i] = sum_section(arrayPartition[i])

    # Returns which partition is the greatest depth wise
    if sumPartition[0] > sumPartition[1] and sumPartition[0] > sumPartition[2]:
        return -1

    elif sumPartition[2] > sumPartition[0] and sumPartition[2] > sumPartition[1]:
        return 1

    else:
        return 0


# Sums all the numbers in the section
def sum_section(section):
    sum = 0

    for i in range(len(section)):
        for j in range(len(section[i])):
            sum += section[i][j]

    return sum


if __name__ == "__main__":
    main()
