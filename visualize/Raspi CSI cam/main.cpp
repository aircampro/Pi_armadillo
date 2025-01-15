#include <stdio.h>
#include "common.h"
#include "app.hpp"

#include <stdlib.h>
#include <signal.h>

void abrt_handler(int sig);

int main()
{
	/* to ensure log file is written by redirect */
	setbuf(stdout, NULL);
	setbuf(stderr, NULL);

	if ( signal(SIGINT, abrt_handler) == SIG_ERR ) {
        exit(1);
    }
	
	App *app = new App();

    /* run the app */
	app->run();

	/* program reaches here when e_flag is true from abort handler on ctrl-C (or sigkill) */
	delete app;

	return 0;
}

void abrt_handler(int sig) {
  e_flag = 1;
}