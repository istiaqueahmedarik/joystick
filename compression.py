import zlib

def compress_string(string):
    compressed_data = zlib.compress(string.encode(),9)
    return compressed_data

def decompress_string(compressed_data):
    decompressed_string = zlib.decompress(compressed_data).decode()
    return decompressed_string

original_string = "[1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000]"
compressed_data = compress_string(original_string)
decompressed_string = decompress_string(compressed_data)

print("Original string:", original_string)
print("Compressed data:", compressed_data)
print("Decompressed string:", decompressed_string)

# memory and length comparison
print("Original string length:", len(original_string))
print("Compressed data length:", len(compressed_data))
print("Memory saved: {:.2f}%".format((1 - len(compressed_data) / len(original_string)) * 100))

