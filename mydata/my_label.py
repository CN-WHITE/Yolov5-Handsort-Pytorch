# -*- coding: utf-8 -*-
import xml.etree.ElementTree as ET
import os
from os import getcwd

sets = ['train', 'val', 'test']
classes = ["four_fingers", "hand_with_fingers_splayed", "index_pointing_up", "little_finger", "ok_hand",
           "raised_fist", "raised_hand", "sign_of_the_horns", "three", "thumbup", "victory_hand"]  # 改成自己的类别
abs_path = os.getcwd()
print(abs_path)


def convert(size, box):
    dw = 1. / (size[0])
    dh = 1. / (size[1])
    x = (box[0] + box[1]) / 2.0 - 1
    y = (box[2] + box[3]) / 2.0 - 1
    w = box[1] - box[0]
    h = box[3] - box[2]
    x = x * dw
    w = w * dw
    y = y * dh
    h = h * dh
    return x, y, w, h


def convert_annotation(image_id):
    in_file = open('E:\pycharm\PycharmProjects\handsort-yolov5\yolov5-master\mydata\Annotations\%s.xml' % (image_id), encoding='UTF-8')  # 改成自己数据存放的地址
    out_file = open('E:\pycharm\PycharmProjects\handsort-yolov5\yolov5-master\mydata\labels\%s.txt' % (image_id), 'w')  # 同上
    tree = ET.parse(in_file)
    root = tree.getroot()
    size = root.find('size')
    w = int(size.find('width').text)
    h = int(size.find('height').text)
    for obj in root.iter('object'):
        difficult = obj.find('difficult').text
        cls = obj.find('name').text
        if cls not in classes or int(difficult) == 1:
            continue
        cls_id = classes.index(cls)
        xmlbox = obj.find('bndbox')
        b = (float(xmlbox.find('xmin').text), float(xmlbox.find('xmax').text), float(xmlbox.find('ymin').text),
             float(xmlbox.find('ymax').text))
        b1, b2, b3, b4 = b
        # 标注越界修正
        if b2 > w:
            b2 = w
        if b4 > h:
            b4 = h
        b = (b1, b2, b3, b4)
        bb = convert((w, h), b)
        out_file.write(str(cls_id) + " " + " ".join([str(a) for a in bb]) + '\n')


wd = getcwd()
for image_set in sets:
    if not os.path.exists('E:\pycharm\PycharmProjects\handsort-yolov5\yolov5-master\mydata\labels'):  # 同上
        os.makedirs('E:\pycharm\PycharmProjects\handsort-yolov5\yolov5-master\mydata\labels')  # 同上
    image_ids = open('E:\pycharm\PycharmProjects\handsort-yolov5\yolov5-master\mydata\ImageSets\Main\%s.txt' % (image_set)).read().strip().split()  # 同上
    list_file = open('E:\pycharm\PycharmProjects\handsort-yolov5\yolov5-master\mydata\%s.txt' % (image_set), 'w')  # 同上
    for image_id in image_ids:
        list_file.write(abs_path + 'E:\pycharm\PycharmProjects\handsort-yolov5\yolov5-master\mydata\images\%s.jpg\n' % (image_id))  # 同上
        convert_annotation(image_id)
    list_file.close()