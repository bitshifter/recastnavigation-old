$(BIN)/fastlz.a: $(OBJ)/fastlz.o
	ar -q $@ $(OBJ)/fastlz.o

$(OBJ)/fastlz.o: RecastDemo/Contrib/fastlz/fastlz.c
	cc $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(BIN)/fastlz.a $(OBJ)/fastlz.o

.PHONY: clean
