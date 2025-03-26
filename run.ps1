#can change game.exe into any title of executable *.exe

gcc main.c -o main.exe -O1 -Wall  -std=c99 -Wno-missing-braces -I include/ -L lib/ -lraylib -lopengl32 -lgdi32 -lwinmm
./main.exe