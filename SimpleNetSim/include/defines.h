// Debug flag
#define DEBUG	0

// Prints debugging message dbgString if DEBUG flag is set
void debugPrint(const char *dbgString) {
	if(DEBUG) {
		puts(dbgString);
	}
}
