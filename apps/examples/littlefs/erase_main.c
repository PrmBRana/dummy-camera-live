/****************************************************************************
 * apps/examples/littlefs/erase_main.c
 *
 * NSH shortcut command: 'erase [hk1|hk2|camera|all]'
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>

extern int littlefs_main(int argc, char *argv[]);

int erase_main(int argc, char *argv[])
{
  char *lfs_argv[5];
  lfs_argv[0] = "littlefs";
  lfs_argv[1] = "erase";
  lfs_argv[2] = (argc > 1) ? argv[1] : NULL;
  lfs_argv[3] = (argc > 2) ? argv[2] : NULL;
  lfs_argv[4] = NULL;
  return littlefs_main((argc > 2) ? 4 : ((argc > 1) ? 3 : 2), lfs_argv);
}
