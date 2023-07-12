// user/find.c
#include "kernel/types.h"
#include "kernel/stat.h"
#include "user/user.h"
#include "kernel/fs.h"

void find(char *path, char *fileName)
{
    char buf[128], *p;
    int fd, fd1;
    struct dirent de;
    struct stat st, st1;

    if ((fd = open(path, 0)) < 0)
    {
        fprintf(2, "path error\n");
        return;
    }

    if (fstat(fd, &st) < 0)
    {
        fprintf(2, "path stat failed\n");
        close(fd);
        return;
    }

    switch (st.type)
    {
    case T_FILE:
        fprintf(2, "path error\n");
        return; // 以上部分判断输入路径是否正确
    case T_DIR:
        strcpy(buf, path);
        p = buf + strlen(buf);
        *p++ = '/'; // 保存当前正在搜索目录的完整路径，作为模板输出，新内容都是固定附加在p指针所指位置
        while (read(fd, &de, sizeof(de)) == sizeof(de))
        { // 遍历搜索目录
            if (de.inum == 0)
                continue;
            if (!strcmp(de.name, ".") || !strcmp(de.name, ".."))
            {
                continue;
            }
            memmove(p, de.name, DIRSIZ);
            if ((fd1 = open(buf, 0)) >= 0)
            {
                if (fstat(fd1, &st1) >= 0)
                {
                    switch (st1.type)
                    {
                    case T_FILE:
                        if (!strcmp(de.name, fileName))
                        {
                            printf("%s\n", buf); // 若文件名与目标文件名一致，则输出其完整路径
                        }
                        close(fd1); // 注意及时关闭不用的文件描述符
                        break;
                    case T_DIR:
                        close(fd1);
                        find(buf, fileName); // 若为目录，则递归查找子目录
                        break;
                    case T_DEVICE:
                        close(fd1);
                        break;
                    }
                }
            }
        }
        break;
    }
    close(fd);
}

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        fprintf(2, "Usage:find path fileName\n");
        exit(0);
    }
    find(argv[1], argv[2]);
    exit(0);
}