#! /usr/bin/env python3
from typing import Optional

import numpy as np
import torch
import pyaudio

import rclpy
from rclpy.node import Node
from rclpy.experimental.events_executor import EventsExecutor

from std_msgs.msg import Bool

import torch.nn as nn
from torchvision import models
import torch
import torch.nn.functional as F


def get_model(device): # -> models.resnet.ResNet:
    #model = models.resnet18(weights="ResNet18_Weights.DEFAULT")
    #num_ftrs = model.fc.in_features
    #model.fc = nn.Linear(num_ftrs, 2)
    
    #model = models.mobilenet_v3_small(num_classes=2)
    model = models.efficientnet_b0(weights="DEFAULT")
    #model.classifier[0] = nn.Dropout(p=0.2) #p=0.2 is default
    in_features = model.classifier[1].in_features
    model.classifier[1] = nn.Linear(in_features, 2)
    return model.to(device)

def resample(
    waveform: torch.Tensor, sample_rate: int, target_sample_rate: int
) -> torch.Tensor:
    """
    Resamples a waveform to a target sample rate.

    :param waveform: Waveform
    :type waveform: torch.Tensor
    :param sample_rate: Sample rate of the waveform
    :type sample_rate: int
    :param target_sample_rate: Target sample rate
    :type target_sample_rate: int
    :return: Resampled waveform
    :rtype: torch.Tensor
    """
    channel = 0
    return torchaudio.transforms.Resample(sample_rate, target_sample_rate)(
        waveform[channel, :].view(1, -1)
    )

def convert_waveform_to_spectogram(
    sample_rate: int, chunk: torch.Tensor
) -> torch.Tensor:
    """
    Converts a waveform to a mel spectrogram.

    :param sample_rate: Sample rate of the waveform
    :type sample_rate: int
    :param chunk: Waveform
    :type chunk: torch.Tensor
    :return: Mel spectrogram
    :rtype: torch.Tensor
    """
    # Repeat the mel spectrogram 3 times to match the number of channels to the expected input of the model
    # TODO: Discuss whether to change the model to accept 1 channel
    return (
        torchaudio.transforms.MelSpectrogram(sample_rate, n_mels=64)(chunk)
        .log2()
        .repeat(3, 1, 1)
    )

class Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(3, 6, 5)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(6, 16, 5)
        self.fc1 = nn.Linear(16 * 5 * 5, 120)
        self.fc2 = nn.Linear(120, 84)
        self.fc3 = nn.Linear(84, 10)

    def forward(self, x):
        x = self.pool(F.relu(self.conv1(x)))
        x = self.pool(F.relu(self.conv2(x)))
        x = torch.flatten(x, 1) # flatten all dimensions except batch
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3

CHUNK = 1024
FORMAT = pyaudio.paFloat32
CHANNELS = 1
SAMPLE_RATE = 44100
TARGET_SAMPLE_RATE = 10_000

CHECKPOINT_PATH = "models/blabla.pth" #TODO: wait for model from teammate


class WhistleDetector(Node):

    def __init__(self) -> None:
        super().__init__("whistle_detector")
        self.logger = self.get_logger()
        
        self.whistle_publisher = self.create_publisher(Bool, "whistle_detected", 1)

        # device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.logger.info(f"Using device: {self.device}")

        # load model
        self.model = get_model(self.device)
        self.model.load_state_dict(
            torch.load(CHECKPOINT_PATH, map_location=self.device)
        )
        self.model.to(self.device)
        self.model.eval()

        # audio setup
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=SAMPLE_RATE,
            input=True,
            frames_per_buffer=CHUNK,
        )

        # timer for processing
        self.timer = self.create_timer(0.02, self.process_audio)  # ~50Hz

        self.logger.info("Whistle detector initialized")

    def process_audio(self) -> None:
        try:
            data = self.stream.read(CHUNK, exception_on_overflow=False)
        except Exception as e:10× lighter and faster than EfficientNet
            self.logger.warn(f"Audio read error: {e}")
            return

        audio_np = np.frombuffer(data, dtype=np.float32)

        audio_tensor = torch.from_numpy(audio_np)
        audio_tensor = audio_tensor.unsqueeze(0)

        # resample
        resampled = resample(
            audio_tensor,
            sample_rate=SAMPLE_RATE,
            target_sample_rate=TARGET_SAMPLE_RATE,
        )

        # mel spectrogram
        mel = convert_waveform_to_spectogram(
            SAMPLE_RATE,
            resampled,
        )

        mel[mel.isinf()] = 0

        mel = mel.unsqueeze(0).to(self.device)

        with torch.no_grad():
            output = self.model(mel).squeeze()

        whistle = bool(output[1] > output[0])

        if whistle:
            msg = Bool()
            msg.data = whistle

            self.whistle_publisher.publish(msg)

    def destroy_node(self):
        try:
            self.stream.stop_stream()
            self.stream.close()
            self.p.terminate()
        except Exception:
            pass

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = WhistleDetector()
    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
