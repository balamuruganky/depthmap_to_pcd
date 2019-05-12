#include <iostream>
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <cstring>
#include <string>
#include <pwd.h>

#include "Utils.h"

using namespace std;

namespace Utils
{
	#define STRING_MAX_LENGTH 512

	char* GetFileName(const char* file_extension) {
		time_t rawtime;
		struct tm * timeinfo;
		static char buffer [(STRING_MAX_LENGTH / 2)] = {'\0'};

		time (&rawtime);
		timeinfo = localtime (&rawtime);

		strftime (buffer,(STRING_MAX_LENGTH / 2),"/%F_%I_%M%p_%S",timeinfo);
		sprintf (buffer, "%s.%s", buffer, file_extension);

		return buffer;
	}

	bool CreateDirectory(char* path) {
		return (!mkdir(path, 0755) || errno == EEXIST);
	}

	char* PrepareUniqueFileName(const char* subdir, const char* file_extension) {
		static char buffer [STRING_MAX_LENGTH] = {'\0'};
		char *temp;

		struct passwd* pwd = getpwuid(getuid());
		if (pwd) {
			temp = pwd->pw_dir;
		} else {
			// try the $HOME environment variable
			temp = getenv("HOME");
		}

		if (temp != NULL) {
			sprintf (buffer, "%s", temp);
		}

		if (strlen(buffer) != 0) {
			sprintf (buffer, "%s/%s/", buffer, subdir);
			if (true != CreateDirectory(buffer)) {
				memset(buffer, 0, sizeof(buffer));
			} else {
				sprintf (buffer, "%s%s", buffer, GetFileName(file_extension));
			}
		}

		if (strlen(buffer) != 0) {
			sprintf (buffer, "%s/%s", "./", GetFileName(file_extension));
		}

		return buffer;
	}

	const bool IsFileExists(char* path) {
		return (access(path, R_OK) == 0);
	}
}
