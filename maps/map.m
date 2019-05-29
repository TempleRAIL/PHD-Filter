static_map = imread('maps/simple_room/simple_rooms.pgm');
imshow(static_map);
hold on;
chair = [31.13 3.53 0.00;
        16.72 18.98 0.00;
        20.62 22.18 0.00;
        11.98 10.93 0.00;
        14.18 19.52 0.00;
        28.02 25.52 0.00;
        35.93 19.77 0.00;
        24.12 8.67 0.00;
        18.02 5.03 0.00;
        30.02 12.67 0.00;
        15.33 3.53 0.00;
        24.72 18.98 0.00
        3.62 5.18 0.00;
        31.18 24.93 0.00;
        15.18 23.52 0.00;
        37.02 2.52 0.00;
        18.93 24.37 0.00;
        37.12 25.67 0.00;
        4.02 21.73 0.00;
        30.02 21.67 0.00;
        4.13 26.53 0.00;
        14.72 14.98 0.00;
        14.62 25.18 0.00;
        14.98 4.93 0.00;
        25.18 14.52 0.00;
        36.02 4.52 0.00;
        29.93 4.77 0.00;
        36.12 23.67 0.00;
        8.82 25.03 0.00;
        24.02 3.67 0.00];
x = 10*chair(:,1);
y = 10*chair(:,2);
plot(x, y, 'g*', 'LineWidth', 0.3, 'MarkerSize', 7);
hold on;
table = [21.03 5.22 0.00;
        35.28 16.18 0.00;
        33.08 13.72 0.00;
        33.33 16.38 0.00;
        4.33 24.98 0.00;
        7.38 26.03 0.00;
        33.38 3.88 0.00;
        4.53 10.48 0.00;
        33.03 10.48 0.00;
        8.12 15.23 0.00;
        33.41 6.13 0.00;
        22.72 10.98 0.00;
        14.62 25.18 0.00;
        13.18 15.35 0.00;
        5.18 2.52 0.00;
        15.02 16.52 0.00;
        11.93 26.37 0.00;
        6.12 24.67 0.00;
        3.12 26.03 0.00;
        24.02 6.67 0.00;
        12.13 19.53 0.00;
        16.72 13.98 0.00;
        11.62 21.18 0.00;
        9.98 14.93 0.00;
        17.98 26.52 0.00;
        27.62 22.22 0.00;
        24.23 20.77 0.00;
        34.12 8.67 0.00;
        15.82 22.73 0.00;
        21.07 15.62 0.00];
x = 10*table(:,1);
y = 10*table(:,2);
plot(x, y, 'b*', 'LineWidth', 0.3, 'MarkerSize', 7);
hold on;
person = [22.23 4.12 0.00;
            27.58 24.53 0.00;
            3.35 15.41 0.00;
            27.43 14.38 0.00;
            34.03 7.23 0.00;
            15.55 6.60 0.00;
            5.91 20.75 0.00;
            33.60 26.49 0.00;
            8.91 5.94 0.00;
            21.02 8.64 0.00;
            16.51 6.12 0.00;
            23.72 2.18 0.00;
            24.62 27.51 0.00;
            32.68 15.35 0.00;
            15.58 21.52 0.00;
            35.02 13.69 0.00;
            26.93 19.37 0.00;
            6.12 24.67 0.00;
            25.12 16.03 0.00;
            6.72 26.67 0.00;
            15.73 21.53 0.00;
            26.72 20.98 0.00;
            17.72 15.18 0.00;
            5.98 21.93 0.00;
            26.98 16.52 0.00;
            35.62 25.22 0.00;
            21.23 16.77 0.00;
            32.12 7.67 0.00;
            16.82 11.73 0.00;
            32.07 10.62 0.00];
x = 10*person(:,1);
y = 10*person(:,2);
plot(x, y, 'r*', 'LineWidth', 0.3, 'MarkerSize', 7);