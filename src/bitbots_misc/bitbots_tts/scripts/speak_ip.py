#!/usr/bin/env python3
"""Script to speak out all relevant IP addresses and their networks."""

import argparse
import os


def get_relevant_ip_addresses() -> list[tuple[str, str]]:
    """
    Discover relevant IP addresses and their networks.

    Returns:
        List of tuples (ip_address, network_name)
    """
    import json
    import subprocess

    ip_addresses = []

    try:
        # Get interface info in JSON format
        result = subprocess.run(["ip", "-j", "addr"], capture_output=True, text=True, check=True)
        interfaces = json.loads(result.stdout)
    except (subprocess.CalledProcessError, json.JSONDecodeError, FileNotFoundError) as e:
        print(f"Error getting IP addresses: {e}")
        return ip_addresses
    print(f"Discovered interfaces: {[iface.get('ifname') for iface in interfaces]}")

    # Build a map of wireless interfaces to their connection names
    wireless_connection_map = {}
    try:
        nmcli_result = subprocess.run(
            ["nmcli", "connection", "show", "--active"], capture_output=True, text=True, check=True
        )
        for line in nmcli_result.stdout.strip().split("\n"):
            if line:
                parts = line.split()
                if len(parts) >= 2:
                    conn_name = parts[0]
                    device_name = parts[-1] if len(parts) > 1 else None
                    if device_name:
                        wireless_connection_map[device_name] = conn_name
    except (subprocess.CalledProcessError, FileNotFoundError):
        # nmcli not available or no active connections, continue without it
        pass
    # Process each interface
    for interface in interfaces:
        ifname = interface.get("ifname")
        # Skip loopback
        if ifname == "lo":
            continue
        # Skip docker
        if ifname and ifname.startswith("docker"):
            continue

        print(
            f"Processing interface: {ifname}"
            f" with addresses: {[addr.get('local') for addr in interface.get('addr_info', [])]}"
        )

        # Look for global scope addresses
        for addr_info in interface.get("addr_info", []):
            if addr_info.get("scope") == "global" and addr_info.get("family") == "inet":
                ip_address = addr_info.get("local")
                if ip_address:
                    # For wireless interfaces, try to get the connection name from nmcli
                    if ifname in wireless_connection_map:
                        network_name = wireless_connection_map[ifname]
                    else:
                        network_name = ifname

                    ip_addresses.append((ip_address, network_name))

    return ip_addresses


def format_ip_for_speech(ip_address: str, network_name: str) -> str:
    """
    Format an IP address for natural speech.

    Args:
        ip_address: IP address string (e.g., "192.168.1.100")
        network_name: Name of the network interface

    Returns:
        Formatted string for speech (e.g., "On interface ethernet, my IP address is 192 dot 168 dot 1 dot 100")
    """
    # Replace dots with spaces for natural speech
    ip_parts = ip_address.replace(".", " dot ")
    # ip_parts = " ".join(list(ip_parts))
    return f"On interface {network_name}, my IP address is {ip_parts}"


def speak_with_espeak(text: str, silent: bool = False) -> None:
    """
    Speak text using espeak command-line tool.

    Args:
        text: Text to speak
        silent: If True, skip audio output but still test the code flow
    """
    if silent:
        print(f"[SILENT] Would speak: {text}")
        return

    import subprocess

    try:
        subprocess.run(["espeak", "-s", "150", text], check=True)
        print(f"Finished speaking: {text}")
    except FileNotFoundError:
        print("Error: espeak not found. Please install espeak: apt-get install espeak")
    except subprocess.CalledProcessError as e:
        print(f"Error running espeak: {e}")


def speak_with_tts(ip_addresses: list[tuple[str, str]]) -> None:
    """
    Speak IP addresses using the TTS engine.

    Currently not in use by default because the diffusion model really struggles with speaking IP addresses and the result are unintelligible. But it can be enabled with --use-tts for fun.

    Args:
        ip_addresses: List of tuples (ip_address, network_name)
    """
    from functools import partial

    import numpy as np
    import soundcard as sc
    from bitbots_tts.supertonic.helper import load_text_to_speech, load_voice_style

    # Load the TTS model
    conda_prefix = os.environ.get("CONDA_PREFIX", "")
    if not conda_prefix:
        raise ValueError("CONDA_PREFIX environment variable not set! We expect models to be shared as conda packages.")

    # Assemble model package name and look at its share directory
    model_path = os.path.join(conda_prefix, "share", "tts_supertonic")

    text_to_speech_engine = load_text_to_speech(os.path.join(model_path, "onnx"), use_gpu=True)

    # Configure voice and diffusion steps
    voice = "F2"
    steps = 10  # Number of diffusion steps, higher is better quality but also slower

    style = load_voice_style([os.path.join(model_path, "voice_styles", f"{voice}.json")])

    generate_speech = partial(text_to_speech_engine, style=style, total_step=steps)

    # Speak each IP address using TTS
    for ip_address, network_name in ip_addresses:
        text = format_ip_for_speech(ip_address, network_name)

        try:
            wav_untrimmed, duration = generate_speech(f"{text}")
            wav = wav_untrimmed[0, : int(text_to_speech_engine.sample_rate * duration[0].item())]
            wav = np.concatenate([np.zeros(2000), wav])
            speaker = sc.default_speaker()
            with speaker.player(samplerate=text_to_speech_engine.sample_rate) as p:
                p.play(wav)
            print(f"Finished speaking: {text} (Duration: {duration[0].item():.2f}s) Used device: {speaker.name}")
        except OSError as e:
            print(f"Error playing audio: {e}")


def main() -> None:
    """Main entry point for the script."""
    parser = argparse.ArgumentParser(description="Speak out all relevant IP addresses.")
    parser.add_argument(
        "--use-tts", action="store_true", help="Use the TTS engine instead of espeak (default is espeak)"
    )
    parser.add_argument("--silent", action="store_true", help="Skip audio output but still test the code flow")
    args = parser.parse_args()

    # Discover IP addresses
    ip_addresses = get_relevant_ip_addresses()

    if not ip_addresses:
        print("No IP addresses found")
        return

    if args.use_tts:
        speak_with_tts(ip_addresses)
    else:
        # Speak each IP address using espeak
        for ip_address, network_name in ip_addresses:
            text = format_ip_for_speech(ip_address, network_name)
            speak_with_espeak(text, silent=args.silent)


if __name__ == "__main__":
    main()
