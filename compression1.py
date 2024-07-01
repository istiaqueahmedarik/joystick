import unishox2
import random
from re import match
# the string we want to compress
original_data = "AAAAABBBCAVAADFHGVHGVHBHGBHGBHGBHBHGVBHGVHBHHGBHJHJBJHN"*10

def compress(inputString):
	"""compression algorithm:
		- The input is a string, and the output is a compressed string.
		- A valid input consists of zero or more upper case english letters A-Z.
		- Any run of two or more of the same character converted to two of that 
		  character plus a number indicating how many repeated runs were compressed. 
		- Only one digit used at a time
		Examples:
		    A --> A
		    AA --> AA0
		    AAA --> AA1
		    AAAAAAAAAAAA --> AA9A
		    AAAAAAAAAAAAA --> AA9AA0
	"""
	outputString = ''
	lastChar = ''
	charIndex = 0
	maxIndex = len(inputString)

	while charIndex < maxIndex:

		nextChar = inputString[charIndex]
		assert(match('[A-Z]', nextChar))
		outputString += nextChar
		charIndex += 1

		if(nextChar == lastChar): # fastforward charIndex through the duplicated characters
			count = 0 # counts duplicate characters compressed
			while((charIndex+count<maxIndex) and (inputString[charIndex+count]==lastChar) and (count<9)):
				count += 1
			charIndex += count
			nextChar = str(count)
			outputString += nextChar
		lastChar = nextChar

	return outputString

def decompress(inputString):
	"""Counterpart to compress -- undoes compression
		inputString=: decompress(compress(inputString))
	"""
	outputString = ''
	lastChar = ''
	charIndex = 0
	maxIndex = len(inputString)

	while charIndex < maxIndex:
		nextChar = inputString[charIndex]

		if(match('[A-Z]', nextChar)):
			outputString += nextChar
		else:
			assert(match('[0-9]',nextChar) and charIndex>1 and match('[A-Z]', lastChar) and (lastChar==inputString[charIndex-2]))
			for i in range(int(nextChar)): outputString += lastChar

		charIndex += 1
		lastChar = nextChar

	return outputString


compressed_data = compress(original_data)
decompressed_data = decompress(compressed_data)

print("Original data:", original_data)
print("Compressed data:", compressed_data)
print("Decompressed data:", decompressed_data)

# memory and length comparison
print("Original data length:", len(original_data))
print("Compressed data length:", len(compressed_data))
print("Memory saved: {:.2f}%".format((1 - len(compressed_data) / len(original_data)) * 100))
