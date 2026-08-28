import requests
import base64
from plugin.signature_interface import Interface

# from cert_dbg_util_data import *
class HTTPInterface(Interface):
    """ Ambiq Signature Provider for remote signing service. """

    def __init__(self, **kwargs) -> None:
        """Initialize the HTTP signature provider.
        :param keyIdx: index of the key to use
        """
        self.url = kwargs["url"]
        self.userID = kwargs["user-id"]
        self.password = kwargs["password"]

    def sign(self, keyIdx: str, dataIn: bytes) -> bytes:
        """Return the signature for the given data.
        :param dataIn: Data to sign
        :return: Signature as bytes object
        """
        # Ensure keyIdx are of type integer if not already
        if type(keyIdx) is not int:
            keyIdx = int(keyIdx, 0)

        endpoint = "{url}/sign/{keyIdx}".format(url=self.url, keyIdx=keyIdx)
        response = requests.put(endpoint, data=dataIn, timeout=30)
        signature = response.content

        return signature

