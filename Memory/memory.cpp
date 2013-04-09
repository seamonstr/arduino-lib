/************
  
 ************/
#include <assert.h>

#include "Arduino.h"
#include "EEPROM.h"
#include "memory.h"

namespace Memory {

  // Move to the beginning of the specified file
  void Memory::open(int fileNo) {
    FileRec fileRec;
    readFileRec(fileNo, &fileRec);
    // Set the address to the beginning of the file
    _currFileStart = fileRec.address;
    _currOffset = 0;
  }

  void Memory::readFileRec(int fileNo, FileRec* fileRec) {
    assert(fileNo < FILE_COUNT);
    // Read the file rec
    seekToFilerec(fileNo);
    read(fileRec, sizeof(FileRec));  
  }

  void Memory::seekToFilerec(int fileNo) {
    _currFileStart = 0;
    _currOffset = (fileNo * sizeof(FileRec));
  }

  void Memory::addFile(int fileno, int length) {
    assert(fileno < FILE_COUNT);
    // Read the record before to work out where to start the new file
    FileRec prevRec;
    if (fileno == 0) {
      prevRec.len = FILE_COUNT * sizeof(FileRec);
      prevRec.address = 0;
    } else {
      readFileRec(fileno - 1, &prevRec);
    }

    // Put together the new record and write it
    FileRec rec;
    rec.len = length;
    rec.address = prevRec.address + prevRec.len;
    seekToFilerec(fileno);
    write(&rec, sizeof(FileRec));
  }

    // Standard r/w stuff
  void Memory::seek(int offset) {
    _currOffset = offset;
  }

  void Memory::read(void* buffer, int len) {
    for (int i = 0; i < len; i ++) {
      ((byte*)buffer)[i] = EEPROM.read(_currFileStart + _currOffset);
      _currOffset ++;
    }
  }

  void Memory::write(void* data, int len) {
    for (int i = 0; i < len; i ++) {
      EEPROM.write(_currFileStart + _currOffset, ((byte*)data)[i]);
      _currOffset ++;
    }
  }

};
