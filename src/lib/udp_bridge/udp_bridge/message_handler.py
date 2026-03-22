import io
import pickle
import zlib

from udp_bridge.aes_helper import AESCipher


class RestrictedUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        raise pickle.UnpicklingError("pickle loading restricted to base types")


class MessageHandler:
    def __init__(self, encryption_key: str | None):
        self.cipher = AESCipher(encryption_key)

    def encrypt_and_encode(self, data: dict) -> bytes:
        serialized_data = zlib.compress(pickle.dumps(data, pickle.HIGHEST_PROTOCOL))
        return self.cipher.encrypt(serialized_data)

    def decrypt_and_decode(self, msg: bytes):
        decrypted_msg = self.cipher.decrypt(msg)
        binary_msg = zlib.decompress(decrypted_msg)
        return RestrictedUnpickler(io.BytesIO(binary_msg)).load()
