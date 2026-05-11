from udp_bridge import aes_helper


def test_decrypt_inverts_encrypt():
    message = b"Hello, World!"
    encrypted = aes_helper.AESCipher("key").encrypt(message)
    decrypted = aes_helper.AESCipher("key").decrypt(encrypted)

    assert message == decrypted
