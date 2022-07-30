#!/usr/bin/env python3

from PIL import Image, ImageFont, ImageDraw

# use a truetype font (.ttf)
# font file from fonts.google.com (https://fonts.google.com/specimen/Courier+Prime?query=courier)
font_path = "fonts/"
font_name = "lucon.ttf"
out_path = font_path

font_size = 12 # px
font_color = "#000000" # HEX Black
font_color = "#FFFFFF" # HEX Black

# Create Font using PIL
font = ImageFont.truetype(font_path+font_name, font_size)

# Copy Desired Characters from Google Fonts Page and Paste into variable
desired_characters = "ABCČĆDĐEFGHIJKLMNOPQRSŠTUVWXYZŽabcčćdđefghijklmnopqrsštuvwxyzž1234567890‘?’“!”(%)[#]{@}/&\<-+÷×=>®©$€£¥¢:;,.*"

img = Image.new("L", (font_size, font_size*256))
draw = ImageDraw.Draw(img)

# Loop through the characters needed and save to desired location
for character in range(0, 255):
    
    # Get text size of character
    width, height = font.getsize(chr(character))
    if width > font_size or height > font_size:
        print(f"char {character} oversize {width}*{height}")

    # Create PNG Image with that size
#    img = Image.new("RGBA", (width, height))
#    draw = ImageDraw.Draw(img)
    
    # Draw the character
    draw.text((0, character * font_size), chr(character), font=font, fill=font_color)
    
    # Save the character as png
try:
    img.save(out_path + "font.png")
except:
    print(f"[-] Couldn't Save:\t{character}")

pix = img.getdata()
#for i in range(0, font_size):
#    for j in range(0, font_size):
#        print(f"img {pix[40*font_size*font_size + i*font_size + j]}")

font_cpp_str = """// generated with font_gen.py

#include <kernel/drivers/font.h>

uint8_t font_greyscale[] = {"""

for i in range(0, len(list(pix))):
    if i % font_size == 0:
        font_cpp_str += "\n"
    if i % (font_size*font_size) == 0:
        font_cpp_str += "\n"
    font_cpp_str += str(pix[i]) + ", "
font_cpp_str += "\n};\n"

font_cpp = open("font.c", "w")
font_cpp.write(font_cpp_str)
font_cpp.close()

font_h_str = f"""#include <kernel/kernel_common.h>

#ifndef _FONT_H_
#define _FONT_H_

#define kCharacterWidth {font_size}
#define kCharacterHeight {font_size}

extern uint8_t font_greyscale[];

#endif"""

font_h = open("font.h", "w")
font_h.write(font_h_str)
font_h.close()

