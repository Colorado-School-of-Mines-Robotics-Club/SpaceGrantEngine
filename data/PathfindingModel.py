from __future__ import annotations

import torch
import kornia
from oakutils.blobs.definitions import AbstractModel, InputType, ModelType
from typing_extensions import Self


class CustomU8(AbstractModel):
    """nn.Module wrapper for a custom operation."""

    def __init__(self: Self) -> None:
        """Create a new instance of the model."""
        super().__init__()

    @classmethod
    def model_type(cls: type[CustomU8]) -> ModelType:
        """Type of input this model takes."""
        return ModelType.NONE

    @classmethod
    def input_names(cls: type[CustomU8]) -> list[tuple[str, InputType]]:
        """Names of the input tensors."""
        return [("input", InputType.U8)]

    @classmethod
    def output_names(cls: type[CustomU8]) -> list[str]:
        """Names of the output tensors."""
        return ["output"]

    def forward(self: Self, image: torch.Tensor) -> torch.Tensor:
        """Forward pass of the model."""
        # TODO: Fill in with custom functionality and compile
        # When compiling with the version of openvino used by default for U8 inputs
        # the network must not be an identity network (i.e. it must do something to the input)
        image = image.float()
        # input shape is [1, 1, H, W]
        print(f"Input shape: {image.shape}")
        image = kornia.filters.gaussian_blur2d(image, (21, 21), (1.5, 1.5))
        _, _, height, width = image.shape
        # trim the far edges of the image
        image = image[:, :, 20 : height - 20, 20 : width - 20]
        _, _, height, width = image.shape
        # create a sub image for for the bottom half, middle half, top half
        quarter = height // 4
        sub1 = image[:, :, 0 : height - 2 * quarter, :]
        sub2 = image[:, :, quarter : height - quarter, :]
        sub3 = image[:, :, 2 * quarter : height, :]

        # split each sub into 5 parts
        fifth = width // 5
        print(fifth)
        sub11 = kornia.filters.gaussian_blur2d(sub1[:, :, :, 0 : fifth], (21, 21), (1.5, 1.5))
        sub12 = kornia.filters.gaussian_blur2d(sub1[:, :, :, fifth : 2 * fifth], (21, 21), (1.5, 1.5))
        sub13 = kornia.filters.gaussian_blur2d(sub1[:, :, :, 2 * fifth : 3 * fifth], (21, 21), (1.5, 1.5))
        sub14 = kornia.filters.gaussian_blur2d(sub1[:, :, :, 3 * fifth : 4 * fifth], (21, 21), (1.5, 1.5))
        sub15 = kornia.filters.gaussian_blur2d(sub1[:, :, :, 4 * fifth : width], (21, 21), (1.5, 1.5))
        sub21 = kornia.filters.gaussian_blur2d(sub2[:, :, :, 0 : fifth], (21, 21), (1.5, 1.5))
        sub22 = kornia.filters.gaussian_blur2d(sub2[:, :, :, fifth : 2 * fifth], (21, 21), (1.5, 1.5))
        sub23 = kornia.filters.gaussian_blur2d(sub2[:, :, :, 2 * fifth : 3 * fifth], (21, 21), (1.5, 1.5))
        sub24 = kornia.filters.gaussian_blur2d(sub2[:, :, :, 3 * fifth : 4 * fifth], (21, 21), (1.5, 1.5))
        sub25 = kornia.filters.gaussian_blur2d(sub2[:, :, :, 4 * fifth : width], (21, 21), (1.5, 1.5))
        sub31 = kornia.filters.gaussian_blur2d(sub3[:, :, :, 0 : fifth], (21, 21), (1.5, 1.5))
        sub32 = kornia.filters.gaussian_blur2d(sub3[:, :, :, fifth : 2 * fifth], (21, 21), (1.5, 1.5))
        sub33 = kornia.filters.gaussian_blur2d(sub3[:, :, :, 2 * fifth : 3 * fifth], (21, 21), (1.5, 1.5))
        sub34 = kornia.filters.gaussian_blur2d(sub3[:, :, :, 3 * fifth : 4 * fifth], (21, 21), (1.5, 1.5))
        sub35 = kornia.filters.gaussian_blur2d(sub3[:, :, :, 4 * fifth : width], (21, 21), (1.5, 1.5))

        # average over the columns
        # shape: [1, 1, W]
        avgdepth11 = torch.mean(sub11, dim=2)
        avgdepth12 = torch.mean(sub12, dim=2)
        avgdepth13 = torch.mean(sub13, dim=2)
        avgdepth14 = torch.mean(sub14, dim=2)
        avgdepth15 = torch.mean(sub15, dim=2)
        avgdepth21 = torch.mean(sub21, dim=2)
        avgdepth22 = torch.mean(sub22, dim=2)
        avgdepth23 = torch.mean(sub23, dim=2)
        avgdepth24 = torch.mean(sub24, dim=2)
        avgdepth25 = torch.mean(sub25, dim=2)
        avgdepth31 = torch.mean(sub31, dim=2)
        avgdepth32 = torch.mean(sub32, dim=2)
        avgdepth33 = torch.mean(sub33, dim=2)
        avgdepth34 = torch.mean(sub34, dim=2)
        avgdepth35 = torch.mean(sub35, dim=2)

        # index of the maximum value in the column vector
        # shape: [1, 1]
        colmax11 = torch.argmax(avgdepth11, dim=2)
        colmax12 = torch.argmax(avgdepth12, dim=2)
        colmax13 = torch.argmax(avgdepth13, dim=2)
        colmax14 = torch.argmax(avgdepth14, dim=2)
        colmax15 = torch.argmax(avgdepth15, dim=2)
        colmax21 = torch.argmax(avgdepth21, dim=2)
        colmax22 = torch.argmax(avgdepth22, dim=2)
        colmax23 = torch.argmax(avgdepth23, dim=2)
        colmax24 = torch.argmax(avgdepth24, dim=2)
        colmax25 = torch.argmax(avgdepth25, dim=2)
        colmax31 = torch.argmax(avgdepth31, dim=2)
        colmax32 = torch.argmax(avgdepth32, dim=2)
        colmax33 = torch.argmax(avgdepth33, dim=2)
        colmax34 = torch.argmax(avgdepth34, dim=2)
        colmax35 = torch.argmax(avgdepth35, dim=2)

        def aggregate(colsub1: torch.Tensor, colsub2: torch.Tensor, colsub3: torch.Tensor) -> torch.Tensor:
            """Aggregate the column max values."""
            colsub1 = torch.multiply(colsub1, 1.0)
            colsub2 = torch.multiply(colsub2, 0.66)
            colsub3 = torch.multiply(colsub3, 0.33)
            colsub = torch.add(torch.add(colsub1, colsub2), colsub3)
            colsub = torch.divide(colsub, 3.0)
            return colsub

        # aggregate each column across the sub images
        # shape: [1, 1]
        col1 = aggregate(colmax11, colmax21, colmax31)  # left
        col2 = aggregate(colmax12, colmax22, colmax32)
        col3 = aggregate(colmax13, colmax23, colmax33)
        col4 = aggregate(colmax14, colmax24, colmax34)
        col5 = aggregate(colmax15, colmax25, colmax35)  # right

        # aggregate the columns
        # shape: [1, 1]
        overall = torch.cat((col1, col2, col3, col4, col5), dim=1)
        colmax = torch.argmax(overall, dim=1)

        print(f"colmax: {colmax}")

        return colmax

