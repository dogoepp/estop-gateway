import hashlib
import hmac

key = "08:36:55"
message = "message"

h = hmac.new(key, message, hashlib.sha256)
result = h.digest()
print(":".join("{:02x}".format(ord(c)) for c in result))
print("HMAC size in bytes : {0}".format(len(result)))