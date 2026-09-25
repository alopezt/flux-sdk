/*
 *---------------------------------------------------------------------------------
 *
 * Copyright (c) 2022-2024, SparkFun Electronics Inc.
 *
 * SPDX-License-Identifier: MIT
 *
 *---------------------------------------------------------------------------------
 */

#include "flxFileRotate.h"
#include "flxClock.h"
#include "flxUtils.h"

#include <Arduino.h>
#include <cstring>

// number of writes between flushes
const int kFlushIncrement = 2;

bool flxFileRotate::getNextFilename(std::string &strFile)
{
    // FS Set?
    if (!_theFS)
        return false;

    char szBuffer[64];
    while (true)
    {
        snprintf(szBuffer, sizeof(szBuffer), kFileNameTemplate, filePrefix.get().c_str(), _currentFileNumber(),
                 kLogFileSuffix);

        // - does this file already exist?
        if (!_theFS->exists(szBuffer))
            break; // free
        _currentFileNumber = _currentFileNumber() + 1;
    }
    strFile = szBuffer;

    return true;
}

//------------------------------------------------------------------------------------------------
// called to open the current log file
bool flxFileRotate::openLogFile(bool bAppend)
{
    if (flxIsLoggingVerbose())
        flxLog_V(F("Opening File: %s, Append Mode: %u"), _currentFilename.c_str(), bAppend);

    _currentFile = _theFS->open(_currentFilename.c_str(),
                                bAppend ? flxIFileSystem::kFileAppend : flxIFileSystem::kFileWrite, true);

    if (!_currentFile)
    {
        flxLogM_E(kMsgErrFileOpen, name(), _currentFilename.c_str());
        return false;
    }
    _flushCount = 0; // new file, new start
    _sizeAtLastFlush = 0;

    return true;
}
//------------------------------------------------------------------------------------------------
// called when we don't have a file open to see if an existing file meets our needs
//

bool flxFileRotate::openCurrentFile(void)
{
    // FS Set?
    if (!_theFS)
        return false;

    char szBuffer[64];

    snprintf(szBuffer, sizeof(szBuffer), kFileNameTemplate, filePrefix.get().c_str(), _currentFileNumber(),
             kLogFileSuffix);

    _currentFilename = szBuffer;

    // If the file exists, no need to add a header.
    bool bExists = _theFS->exists(szBuffer);
    if (bExists)
        _headerWritten = true;

    return openLogFile(bExists); // send in true for append mode if file exists.
}
//------------------------------------------------------------------------------------------------
// GURT-1: when a sector write fails inside FatFs's f_write, every later write to that open file fails
// too (a sticky error that only reopening clears), and flush() reports nothing. The failure shows only
// as a log file that stopped growing. size() reads the directory entry, which every successful flush
// updates.
bool flxFileRotate::logFileStoppedGrowing(void)
{
    size_t size = _currentFile.size();
    bool stoppedGrowing = size <= _sizeAtLastFlush;
    _sizeAtLastFlush = size;

    return stoppedGrowing;
}

//------------------------------------------------------------------------------------------------
// GURT-1: close the log file; the next write() reopens the same file in append mode.
void flxFileRotate::closeLogFileAfterWriteError(void)
{
    flxLog_W(F("%s: log file %s stopped growing - reopening it"), name(), _currentFilename.c_str());

    _currentFile.close();
    _currentFile = flxFSFile(); // "null file"
    _reopenAfterWriteError = true;
}

//------------------------------------------------------------------------------------------------
// Open the next log file.

bool flxFileRotate::openNextLogFile()
{

    if (_currentFile)
    {
        _currentFile.close();
        _currentFile = flxFSFile(); // "null file"
        _currentFilename = "";
    }
    // Open the next file
    std::string nextFile;
    if (!getNextFilename(nextFile))
        return false;

    _currentFilename = nextFile;

    if (!openLogFile())
        return false;

    _secsFileOpen = flxClock.epoch();

    // no header written to file yet...
    _headerWritten = false;

    // send the new file event. Will persist prop values
    flxSendEvent(flxEvent::kOnNewFile);

    return true;
}

//------------------------------------------------------------------------------------------------
void flxFileRotate::write(int32_t value)
{
    write(flx_utils::to_string(value).c_str(), true, flxLineTypeData);
}

//------------------------------------------------------------------------------------------------
void flxFileRotate::write(float value)
{
    write(flx_utils::to_string(value).c_str(), true, flxLineTypeData);
}

//------------------------------------------------------------------------------------------------
void flxFileRotate::write(const char *value, bool newline, flxLineType_t type)
{
    // Mirror every telemetry data record out UART2 TX (GPIO17, 115200 8N1,
    // RX unmapped) for an off-board downlink. This is the same record the SD
    // card receives, in whatever format (JSON, CSV, ...) is configured - the
    // formatter has already serialized it before calling write(). Done before
    // the SD-state checks so the downlink keeps flowing with no card inserted.
    if (type == flxLineTypeData && value)
    {
        static bool s_uart2MirrorStarted = false;
        if (!s_uart2MirrorStarted)
        {
            // An 8 KB TX ring buffer must be installed before begin(). With it,
            // the IDF UART driver drains to the wire in the background, so
            // write() just copies into the buffer instead of blocking the
            // logger at line rate (the default txBufferSize==0 path blocks
            // until bytes clock out of the 128-byte hardware FIFO).
            Serial2.setTxBufferSize(8192);
            Serial2.begin(115200, SERIAL_8N1, -1, 17);
            s_uart2MirrorStarted = true;
        }
        // Best-effort, non-blocking: uart_write_bytes() blocks until the whole
        // payload fits the ring buffer, so only emit when the record + newline
        // already fits the free space. A slow/absent downstream link drops
        // telemetry records here; it never backpressures logging or SD writes.
        size_t len = strlen(value);
        if (Serial2.availableForWrite() >= (int)(len + 1))
        {
            Serial2.write((const uint8_t *)value, len);
            Serial2.write((uint8_t)'\n');
        }
    }

    if (!_theFS)
        return;

    // GURT-1: a log file closed after a write error is reopened in append mode, retrying on every write
    // until that succeeds. Never through the no-file path below: when a read error hides the existing
    // file, that path opens it with "w" and truncates the log.
    if (_reopenAfterWriteError)
    {
        if (!openLogFile(true))
            return;
        _reopenAfterWriteError = false;
    }

    // no file - system just starting up?
    if (!_currentFile)
    {
        bool status;

        // Do we use a current file, or go for the next file?
        //
        // Reasons for Next file:
        //      - No state saved - starting new (_secsFileOpen == 0)
        //      - or if the current elapsed period has expired

        if (_secsFileOpen() == 0 || flxClock.epoch() - _secsFileOpen() > _secsRotPeriod)
            status = openNextLogFile();
        else // have state, use current file
            status = openCurrentFile();

        // error loading
        if (!status)
            return;
    }

    if (flxIsLoggingVerbose())
        flxLog_V(F("Writing to file: %s, value:\"%s\""), _currentFilename.c_str(), value);
    // Data line? write it
    if (type == flxLineTypeData)
    {
        // Write the current line out
        _currentFile.write((uint8_t *)value, strlen(value) + 1);
        // add a cr if newline set
        if (newline)
            _currentFile.write((uint8_t *)"\n", 1);
    }
    // if this is a header line, and we've not written a header, write it
    else if (type == flxLineTypeHeader && !_headerWritten)
    {
        // Write the current line out
        _currentFile.write((uint8_t *)value, strlen(value) + 1);
        // add a cr if newline set
        if (newline)
            _currentFile.write((uint8_t *)"\n", 1);

        _headerWritten = true;
    }

    // Will we need to rotate?
    if (flxClock.epoch() - _secsFileOpen() > _secsRotPeriod)
    {
        // open the next file, send the new file event. This will cause
        // the next line out to be a "start of the file line" (i.e. header)
        // if that's how the format rolls
        if (!openNextLogFile())
            return;
    }

    // flush the file buffer?
    _flushCount = (_flushCount + 1) % kFlushIncrement;
    if (!_flushCount)
    {
        _currentFile.flush();

        if (logFileStoppedGrowing())
            closeLogFileAfterWriteError();
    }
}
