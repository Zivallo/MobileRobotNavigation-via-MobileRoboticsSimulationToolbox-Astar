clc,close,clear;

image = imread('map1.png');

bwimage = image > 0.5;

map = binaryOccupancyMap(bwimage);
show(map)