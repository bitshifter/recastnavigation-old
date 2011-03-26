NAME = DetourTileCache

SOURCES = \
	DetourTileCacheBuilder.cpp

HEADERS = \
	DetourTileCacheBuilder.h

CPPFLAGS = \
	-I Detour/Include \
	-I Recast/Include

$(BIN)/$(NAME).a: $(OBJ)/$(NAME).o
	ar -q $@ $(OBJ)/$(NAME).o

$(OBJ)/$(NAME).o: $(NAME)/DetourTileCacheBuilder.cpp
	c++ $(CPPFLAGS) -c -o $@ $<

clean:
	rm -f $(BIN)/$(NAME).a $(OBJ)/$(NAME).o

.PHONY: clean
