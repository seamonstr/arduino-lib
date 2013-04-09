/***********
 Manages the stuff saved in the EEPROM

File table is:
[file_rec, ...]

File_rec is:
int address
byte len
***********/
#ifndef memory_h
#define memory_h

#define FILE_COUNT 10

namespace Memory {

  struct FileRec {
    int address;
    byte len;
  };

  class Memory {
   protected:
    int _currOffset;
    int _currFileStart;

    void readFileRec(int fileNo, FileRec* fileRec);
    void seekToFilerec(int fileNo);

   public:
    // Move to the beginning of the specified file
    void open(int fileno);

    // Add a new file
    void addFile(int fileno, int length);

    // Standard r/w stuff
    void seek(int offset);
    void read(void* buffer, int len);
    void write(void* data, int len);
  };
    
};

#endif
