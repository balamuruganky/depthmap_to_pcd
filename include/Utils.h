#ifndef UTILS_H
#define UTILS_H

namespace Utils
{
  char *GetFileName ();
  char *PrepareUniqueFileName (const char *subdir,
			       const char *file_extension);
  const bool IsFileExists (char *path);
  bool CreateDirectory (char *path);
}

#endif
