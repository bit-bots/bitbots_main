# -*- coding:utf-8 -*-
"""
MakeCairoShapes
^^^^^^^^^^^^^^^

Helper class for making cairo shapes in the visualizaiton.


History:
''''''''
* 26.04.15. Created(Marc)
"""
import numpy
import cairo


standard_font_size = 12

# todo commentars
def transform_relative_to_pixel(x, y, radius, width, height):
    x /= -1
    y /= -1
    x = x * width
    y = y * width
    x += width * 0.5
    y += height * 0.5
    radius *= width
    return x, y, radius


def ball_shape(x, y, radius, r, g, b, line_width):
    return [['circle', {"radius": radius, "x": x, "y": y, "color.r": r, "color.g": g, "color.b": b, "line": line_width}]]

def white_ball(x, y, radius):
    return ball_shape(x, y, radius, 255, 255, 255, 1)

def red_ball(x, y, radius):
    return ball_shape(x, y, radius, 255, 0, 0, 1)

def yellow_ball(x, y, radius):
    return ball_shape(x, y, radius, 255, 255, 0, 2)

def violet_ball(x, y, radius):
    return ball_shape(x, y, radius, 255, 0, 255, 3)




def text_shape(x, y, r, g, b, font_size, text):
    return [['text', {"x": x, "y": y, "color.r": r, "color.g": g, "color.b": b, "font_size": font_size, "text": text}]]

def red_text(x, y, text):
    return text_shape(x, y, 255, 0, 0, standard_font_size, text)

def big_red_text(x, y, text):
    return text_shape(x, y, 255, 0, 0, 20, text)

def yellow_text(x, y, text):
    return text_shape(x, y, 255, 255, 0, standard_font_size, text)

def green_text(x, y, text):
    return text_shape(x, y, 0, 255, 0, standard_font_size, text)

def blue_text(x, y, text):
    return text_shape(x, y, 0, 0, 255, standard_font_size, text)

def white_text(x, y, text):
    return text_shape(x, y, 255, 255, 255, standard_font_size, text)


def text_block(x, y, r, g, b, font_size, text_array):
    shapes = []
    for text in text_array:
        shapes.extend(text_shape(x, y, r, g, b, font_size, text))
        y += font_size + 5
    return shapes

def white_text_block(x, y, text_array):
    return text_block(x, y, 255, 255, 255, standard_font_size, text_array)

def list_text(x, y, text_list):
    print text_list
    shapes = []
    for text in text_list:
        shapes.extend(red_text(x, y, text[0]))
        y += standard_font_size + 5
        shapes.extend(white_text_block(x, y, text[1:]))
    return shapes

def rectangle(x, y, w, h, r, g, b):
    return [['rect', {"x": x, "y": y, "width": w, "height": h, "color.r": r, "color.g": g, "color.b": b}]]

def white_rectangle(x, y, w, h):
    return rectangle(x, y, w, h, 0, 0, 0)

def yellow_rectangle(x, y, w, h):
    return rectangle(x, y, w, h, 255, 255, 0)

def red_rectangle(x, y, w, h):
    return rectangle(x, y, w, h, 255, 0, 0)


def filled_rectangle_shape(x, y, w, h, r, g, b):
    return [['filled_rect', {"x": x, "y": y, "width": w, "height": h, "color.r": r, "color.g": g, "color.b": b}]]

def black_filled_rect(x, y, w, h):
    return filled_rectangle_shape(x, y, w, h, 0, 0, 0)



def line(x1, y1, x2, y2, r, g, b, line_width):
    return [['line', {"x1": x1, "y1": y1, "x2": x2, "y2": y2, "color.r": r, "color.g": g, "color.b": b, "line": line_width}]]

def green_line(x1, y1, x2, y2):
    return line(x1,y1,x2,y2, 0, 255, 0, 1)

def white_line(x1, y1, x2, y2):
    return line(x1,y1,x2,y2, 255, 255, 255, 1)


def draw_shapes(image, shapes):
    ################################
    # draw the shapes on the image #
    ################################
    height, width = image.shape[:2]

    if len(image.shape) == 2:
        image = numpy.dstack((image, image, image, numpy.empty((height, width), dtype=numpy.uint8))).copy()

    ctx = cairo.Context(cairo.ImageSurface.create_for_data(
        image, cairo.FORMAT_RGB24, width, height, width * 4))

    try:
        for shape in shapes:
            if len(shape) != 2:
                print "KEY ERROR IN SHAPES DICT!!! Failed shape: " + str(shape)
                continue
            stype, values = shape
            if not all(v == v for v in values.values()):
                # NaN rausfiltern
                continue

            if stype == "line":
                ctx.move_to(values["x1"], values["y1"])
                ctx.line_to(values["x2"], values["y2"])
                ctx.set_line_width(values["line"])
                ctx.set_source_rgb(values["color.r"], values["color.g"], values["color.b"])
                ctx.stroke()

            elif stype == "point":
                ctx.rectangle(values["x"] - 1.5, values["y"] - 1.5, 3, 3)
                ctx.set_line_width(values["line"])
                ctx.set_source_rgb(values["color.r"], values["color.g"], values["color.b"])
                ctx.stroke()

            elif stype == "dot":
                ctx.rectangle(int(values["x"]), int(values["y"]), 1, 1)
                ctx.set_source_rgb(values["color.r"], values["color.g"], values["color.b"])
                ctx.fill()

            elif stype == "circle":
                ctx.arc(values["x"], values["y"], values["radius"], 0, 3.14159 * 2)
                ctx.set_line_width(values["line"])
                ctx.set_source_rgb(values["color.r"], values["color.g"], values["color.b"])
                ctx.stroke()

            elif stype == "rect":
                ctx.rectangle(values["x"], values["y"],  values["width"], values["height"])
                ctx.set_line_width(values["line"])
                ctx.set_source_rgb(values["color.r"], values["color.g"], values["color.b"])
                ctx.stroke()

            elif stype == "filled_rect":
                ctx.rectangle(values["x"], values["y"], values["width"], values["height"])
                ctx.set_source_rgb(values["color.r"], values["color.g"], values["color.b"])
                ctx.fill()

            elif stype == "text":
                with SaveCairoState(ctx):
                    ctx.translate(values["x"], values["y"])
                    ctx.set_source_rgb(values["color.r"], values["color.g"], values["color.b"])
                    ctx.set_font_size(values["font_size"])
                    ctx.show_text(values["text"])
                    ctx.stroke()
            else:
                print "######################Wrong type for debug shape ###########"

    except cairo.Error:
        pass

    return image




class SaveCairoState(object):
    def __init__(self, ctx):
        self.ctx = ctx

    def __enter__(self):
        self.ctx.save()
        return self.ctx

    def __exit__(self, *ignore):
        self.ctx.restore()
