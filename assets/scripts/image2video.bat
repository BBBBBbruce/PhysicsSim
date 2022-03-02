ffmpeg.exe -i D:\Bruce_Projects\BBBBBbruce\PhysicsSim\bin\out\Project1642778211\images\Image%04d.png -c:v libx264 -pix_fmt yuv420p -f mp4 -r 60 output.mp4

ffmpeg.exe -framerate 24 -i Image%04d.png -c:v libx264 -pix_fmt yuv420p -crf 23 avideo.mp4