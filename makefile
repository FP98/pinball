MAIN = pinball

CC = gcc
CFLAGS = -Wall
OBJ1 = ptask
OBJS = $(MAIN).o $(OBJ1).o 
LIBS = -lpthread -lrt -lm `allegro-config --libs`
$(MAIN):$(OBJS)
	$(CC) -o $(MAIN) $(OBJS) $(LIBS) $(CFLAGS)
$(MAIN).o: $(MAIN).c
	$(CC) -c $(MAIN).c
$(OBJ1).o: $(OBJ1).c
	$(CC) -c $(OBJ1).c
clean:
	rm -rf *o $(MAIN)