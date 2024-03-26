from torch import nn
import torch


class BasicMobileNetBlock(nn.Module):
    def __init__(self, in_dim, out_dim, leaky_relu_coef=0, dropout=0.3, kernel_size=5):
        super().__init__()
        self.spatial_convolve = nn.Conv2d(
            in_dim,
            in_dim,
            kernel_size,
            padding=kernel_size // 2,
            padding_mode="reflect",
            groups=in_dim,
        )
        self.depthwise_convolve = nn.Conv2d(
            in_dim,
            out_dim,
            kernel_size=1,
        )
        if leaky_relu_coef == 0:
            self.ReLU = nn.ReLU()
        else:
            self.ReLU = nn.LeakyReLU(leaky_relu_coef)
        self.batch_norm = nn.BatchNorm2d(out_dim)
        self.dropout = nn.Dropout2d(p=dropout)

    def forward(self, X):
        _X = self.spatial_convolve(X)
        _X = self.depthwise_convolve(X)
        _X = self.ReLU(_X)
        _X = self.batch_norm(_X)
        return self.dropout(_X)


class MySegmentationNet(nn.Module):
    def __init__(self, block_amount=2, leaky_relu_coef=0, dropout=0.3, kernel_size=5):
        super().__init__()
        self.in_layer = BasicMobileNetBlock(3, 64, leaky_relu_coef=leaky_relu_coef, dropout=dropout,
                                            kernel_size=kernel_size)
        self.basic_layers = nn.Sequential(
            BasicMobileNetBlock(64, 64, leaky_relu_coef=leaky_relu_coef, dropout=dropout, kernel_size=kernel_size),
            BasicMobileNetBlock(64, 64, leaky_relu_coef=leaky_relu_coef, dropout=dropout, kernel_size=kernel_size)
        )
        self.out_layer = nn.Conv2d(64, 3, 1)

    def forward(self, X):
        _X = self.in_layer(X)
        _X = self.basic_layers(_X)
        return self.out_layer(_X)
