#include <stdio.h>
#include <stdlib.h>

int main( int argc, char* argv[] )
{
      FILE *in, *out;
      int ch;  // char

      if( argc != 3 )
      {
            fprintf( stderr, "输入形式: copyFile 源文件名 �?标文件�??1?7 \n" );
            exit( EXIT_FAILURE );
      }

      if( ( in = fopen( argv[1], "rb") ) == NULL )
      {
            fprintf( stderr, "打不开文件: %s \n", argv[1] );
            exit( EXIT_FAILURE );
      }

      if( ( out = fopen( argv[2], "wb") ) == NULL )
      {
            fprintf( stderr, "打不开文件: %s \n", argv[2] );
            fclose( in );                                   // 记得擦屁�?1?7
            exit( EXIT_FAILURE );
      }

      while( (ch = getc(in)) != EOF ) // EOF == end of file
      {
            if( putc( ch, out ) == EOF )
            {
                  break;
            }
      }

      if( ferror( in ) )
      {
            printf("读取文件 %s 失败! \n", argv[1] );
      }

      if( ferror( out ))
      {
            printf("写入文件 %s 失败! \n", argv[2] );
      }

      printf("成功复制1�?文�??1?7!\n");

      fclose( in );
      fclose( out );

      return 0;
}