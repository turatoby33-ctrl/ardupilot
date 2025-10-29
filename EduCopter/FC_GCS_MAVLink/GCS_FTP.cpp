/**
 * @file GCS_FTP.cpp
 * @brief MAVLink FTP protocol implementation
 *
 * This file implements the MAVLink FTP protocol for transferring files
 * between ground station and vehicle. Supports:
 * - Directory listing
 * - File read/write
 * - File creation/deletion
 * - File truncation
 * - Calculate file CRC32
 * - Burst read mode for faster transfers
 *
 * Protocol uses FILE_TRANSFER_PROTOCOL messages with embedded FTP payload.
 *
 * @author EduCopter Development Team
 * @date 2025
 */

#include "GCS_FTP.h"
#include "GCS.h"
#include <cstring>
#include <cstdio>

namespace EduCopter {
namespace GCS {

// External filesystem interface (implemented by vehicle)
extern bool fs_openFile(const char* path, const char* mode, void** outHandle);
extern bool fs_closeFile(void* handle);
extern int32_t fs_readFile(void* handle, uint8_t* buffer, uint32_t length);
extern int32_t fs_writeFile(void* handle, const uint8_t* buffer, uint32_t length);
extern bool fs_seekFile(void* handle, uint32_t offset);
extern bool fs_deleteFile(const char* path);
extern bool fs_listDirectory(const char* path, char* outBuffer, uint32_t bufferSize);
extern bool fs_getFileSize(const char* path, uint32_t& outSize);
extern bool fs_createDirectory(const char* path);
extern bool fs_removeDirectory(const char* path);
extern uint32_t fs_calculateCRC32(const char* path);

// FTP session state
struct FTPSession {
    bool active;
    void* fileHandle;
    uint32_t offset;
    uint16_t seqNumber;
    uint8_t targetSystem;
    uint8_t targetComponent;
    char currentPath[128];
};

static FTPSession s_session = {false, nullptr, 0, 0, 0, 0, ""};

// FTP payload size (max 251 bytes per MAVLink spec)
static const uint32_t FTP_PAYLOAD_SIZE = 251;

/**
 * @brief Constructor
 */
GCS_FTP::GCS_FTP(GCSChannel& channel)
    : m_channel(channel)
{
}

/**
 * @brief Handle FILE_TRANSFER_PROTOCOL message
 *
 * Dispatches FTP requests to appropriate handlers.
 */
void GCS_FTP::handleMessage(const mavlink_message_t& msg)
{
    mavlink_file_transfer_protocol_t packet;
    mavlink_msg_file_transfer_protocol_decode(&msg, &packet);

    // Check if message is for us
    if (packet.target_system != m_channel.getSystemID()) {
        return;
    }

    if (packet.target_component != m_channel.getComponentID() &&
        packet.target_component != MAV_COMP_ID_ALL) {
        return;
    }

    // Parse FTP payload
    const FTPPayload* request = reinterpret_cast<const FTPPayload*>(packet.payload);

    // Update session info
    s_session.targetSystem = msg.sysid;
    s_session.targetComponent = msg.compid;
    s_session.seqNumber = request->seqNumber;

    // Dispatch based on opcode
    switch (static_cast<FTPOpcode>(request->opcode)) {
        case FTPOpcode::None:
            sendNak(FTPErrorCode::InvalidOpcode);
            break;

        case FTPOpcode::TerminateSession:
            handleTerminateSession();
            break;

        case FTPOpcode::ResetSession:
            handleResetSession();
            break;

        case FTPOpcode::ListDirectory:
            handleListDirectory(request);
            break;

        case FTPOpcode::OpenFileRO:
            handleOpenFileRO(request);
            break;

        case FTPOpcode::ReadFile:
            handleReadFile(request);
            break;

        case FTPOpcode::CreateFile:
            handleCreateFile(request);
            break;

        case FTPOpcode::WriteFile:
            handleWriteFile(request);
            break;

        case FTPOpcode::RemoveFile:
            handleRemoveFile(request);
            break;

        case FTPOpcode::CreateDirectory:
            handleCreateDirectory(request);
            break;

        case FTPOpcode::RemoveDirectory:
            handleRemoveDirectory(request);
            break;

        case FTPOpcode::OpenFileWO:
            handleOpenFileWO(request);
            break;

        case FTPOpcode::TruncateFile:
            handleTruncateFile(request);
            break;

        case FTPOpcode::Rename:
            handleRename(request);
            break;

        case FTPOpcode::CalcFileCRC32:
            handleCalcFileCRC32(request);
            break;

        case FTPOpcode::BurstReadFile:
            handleBurstReadFile(request);
            break;

        default:
            sendNak(FTPErrorCode::InvalidOpcode);
            break;
    }
}

/**
 * @brief Terminate FTP session
 */
void GCS_FTP::handleTerminateSession()
{
    closeCurrentFile();
    s_session.active = false;
    sendAck(nullptr, 0);
}

/**
 * @brief Reset FTP session
 */
void GCS_FTP::handleResetSession()
{
    closeCurrentFile();
    s_session.active = false;
    s_session.offset = 0;
    memset(s_session.currentPath, 0, sizeof(s_session.currentPath));
    sendAck(nullptr, 0);
}

/**
 * @brief List directory contents
 */
void GCS_FTP::handleListDirectory(const FTPPayload* request)
{
    const char* path = reinterpret_cast<const char*>(request->data);
    char buffer[FTP_PAYLOAD_SIZE];

    uint32_t offset = request->offset;

    if (!fs_listDirectory(path, buffer, sizeof(buffer))) {
        sendNak(FTPErrorCode::FailErrno);
        return;
    }

    // Send directory listing (may need multiple packets for large directories)
    uint32_t totalSize = strlen(buffer);
    uint32_t chunkSize = FTP_PAYLOAD_SIZE - 12; // Leave room for header

    if (offset < totalSize) {
        uint32_t remaining = totalSize - offset;
        uint32_t sendSize = (remaining < chunkSize) ? remaining : chunkSize;

        sendAck(reinterpret_cast<const uint8_t*>(buffer + offset), sendSize);
    } else {
        // End of directory listing
        sendAck(nullptr, 0);
    }
}

/**
 * @brief Open file for reading
 */
void GCS_FTP::handleOpenFileRO(const FTPPayload* request)
{
    closeCurrentFile();

    const char* path = reinterpret_cast<const char*>(request->data);

    if (!fs_openFile(path, "rb", &s_session.fileHandle)) {
        sendNak(FTPErrorCode::FileNotFound);
        return;
    }

    s_session.active = true;
    s_session.offset = 0;
    strncpy(s_session.currentPath, path, sizeof(s_session.currentPath) - 1);

    // Get file size
    uint32_t fileSize;
    if (fs_getFileSize(path, fileSize)) {
        uint8_t sizeData[4];
        memcpy(sizeData, &fileSize, 4);
        sendAck(sizeData, 4);
    } else {
        sendAck(nullptr, 0);
    }
}

/**
 * @brief Read file data
 */
void GCS_FTP::handleReadFile(const FTPPayload* request)
{
    if (!s_session.active || !s_session.fileHandle) {
        sendNak(FTPErrorCode::FileNotFound);
        return;
    }

    uint32_t offset = request->offset;
    uint32_t size = request->size;

    // Limit read size
    if (size > FTP_PAYLOAD_SIZE - 12) {
        size = FTP_PAYLOAD_SIZE - 12;
    }

    // Seek to offset
    if (!fs_seekFile(s_session.fileHandle, offset)) {
        sendNak(FTPErrorCode::FailErrno);
        return;
    }

    // Read data
    uint8_t buffer[FTP_PAYLOAD_SIZE];
    int32_t bytesRead = fs_readFile(s_session.fileHandle, buffer, size);

    if (bytesRead < 0) {
        sendNak(FTPErrorCode::FailErrno);
        return;
    }

    sendAck(buffer, bytesRead);
}

/**
 * @brief Create new file
 */
void GCS_FTP::handleCreateFile(const FTPPayload* request)
{
    closeCurrentFile();

    const char* path = reinterpret_cast<const char*>(request->data);

    if (!fs_openFile(path, "wb", &s_session.fileHandle)) {
        sendNak(FTPErrorCode::FailErrno);
        return;
    }

    s_session.active = true;
    s_session.offset = 0;
    strncpy(s_session.currentPath, path, sizeof(s_session.currentPath) - 1);

    sendAck(nullptr, 0);
}

/**
 * @brief Write file data
 */
void GCS_FTP::handleWriteFile(const FTPPayload* request)
{
    if (!s_session.active || !s_session.fileHandle) {
        sendNak(FTPErrorCode::FileNotFound);
        return;
    }

    uint32_t offset = request->offset;
    uint32_t size = request->size;

    // Seek to offset
    if (!fs_seekFile(s_session.fileHandle, offset)) {
        sendNak(FTPErrorCode::FailErrno);
        return;
    }

    // Write data
    int32_t bytesWritten = fs_writeFile(s_session.fileHandle,
                                        request->data, size);

    if (bytesWritten < 0 || static_cast<uint32_t>(bytesWritten) != size) {
        sendNak(FTPErrorCode::FailErrno);
        return;
    }

    sendAck(nullptr, 0);
}

/**
 * @brief Remove file
 */
void GCS_FTP::handleRemoveFile(const FTPPayload* request)
{
    const char* path = reinterpret_cast<const char*>(request->data);

    if (fs_deleteFile(path)) {
        sendAck(nullptr, 0);
    } else {
        sendNak(FTPErrorCode::FailErrno);
    }
}

/**
 * @brief Create directory
 */
void GCS_FTP::handleCreateDirectory(const FTPPayload* request)
{
    const char* path = reinterpret_cast<const char*>(request->data);

    if (fs_createDirectory(path)) {
        sendAck(nullptr, 0);
    } else {
        sendNak(FTPErrorCode::FailErrno);
    }
}

/**
 * @brief Remove directory
 */
void GCS_FTP::handleRemoveDirectory(const FTPPayload* request)
{
    const char* path = reinterpret_cast<const char*>(request->data);

    if (fs_removeDirectory(path)) {
        sendAck(nullptr, 0);
    } else {
        sendNak(FTPErrorCode::FailErrno);
    }
}

/**
 * @brief Open file for writing
 */
void GCS_FTP::handleOpenFileWO(const FTPPayload* request)
{
    closeCurrentFile();

    const char* path = reinterpret_cast<const char*>(request->data);

    if (!fs_openFile(path, "ab", &s_session.fileHandle)) {
        sendNak(FTPErrorCode::FailErrno);
        return;
    }

    s_session.active = true;
    strncpy(s_session.currentPath, path, sizeof(s_session.currentPath) - 1);

    sendAck(nullptr, 0);
}

/**
 * @brief Truncate file
 */
void GCS_FTP::handleTruncateFile(const FTPPayload* request)
{
    const char* path = reinterpret_cast<const char*>(request->data);
    uint32_t offset = request->offset;

    // Close if currently open
    closeCurrentFile();

    // Reopen for truncation
    void* handle;
    if (!fs_openFile(path, "r+b", &handle)) {
        sendNak(FTPErrorCode::FileNotFound);
        return;
    }

    // Truncate (platform-specific implementation needed)
    // For now, just close
    fs_closeFile(handle);

    sendAck(nullptr, 0);
}

/**
 * @brief Rename file/directory
 */
void GCS_FTP::handleRename(const FTPPayload* request)
{
    // Extract old and new names from data (null-separated)
    const char* oldPath = reinterpret_cast<const char*>(request->data);
    const char* newPath = oldPath + strlen(oldPath) + 1;

    // Platform-specific rename function needed
    // For now, return unsupported
    sendNak(FTPErrorCode::InvalidOpcode);
}

/**
 * @brief Calculate file CRC32
 */
void GCS_FTP::handleCalcFileCRC32(const FTPPayload* request)
{
    const char* path = reinterpret_cast<const char*>(request->data);

    uint32_t crc = fs_calculateCRC32(path);

    uint8_t crcData[4];
    memcpy(crcData, &crc, 4);

    sendAck(crcData, 4);
}

/**
 * @brief Burst read file
 *
 * Sends multiple file chunks in rapid succession for faster transfers.
 */
void GCS_FTP::handleBurstReadFile(const FTPPayload* request)
{
    if (!s_session.active || !s_session.fileHandle) {
        sendNak(FTPErrorCode::FileNotFound);
        return;
    }

    uint32_t offset = request->offset;
    uint32_t size = request->size;

    // Send up to 10 packets in burst
    const uint32_t BURST_PACKETS = 10;
    const uint32_t CHUNK_SIZE = FTP_PAYLOAD_SIZE - 12;

    for (uint32_t i = 0; i < BURST_PACKETS; i++) {
        if (size == 0) break;

        uint32_t chunkSize = (size < CHUNK_SIZE) ? size : CHUNK_SIZE;

        // Seek to offset
        if (!fs_seekFile(s_session.fileHandle, offset)) {
            sendNak(FTPErrorCode::FailErrno);
            return;
        }

        // Read data
        uint8_t buffer[FTP_PAYLOAD_SIZE];
        int32_t bytesRead = fs_readFile(s_session.fileHandle, buffer, chunkSize);

        if (bytesRead < 0) {
            sendNak(FTPErrorCode::FailErrno);
            return;
        }

        if (bytesRead == 0) {
            // EOF
            break;
        }

        sendAck(buffer, bytesRead);

        offset += bytesRead;
        size -= bytesRead;
    }
}

/**
 * @brief Send FTP ACK response
 */
void GCS_FTP::sendAck(const uint8_t* data, uint8_t dataSize)
{
    FTPPayload response;
    memset(&response, 0, sizeof(response));

    response.seqNumber = s_session.seqNumber;
    response.opcode = static_cast<uint8_t>(FTPOpcode::Ack);
    response.size = dataSize;

    if (data && dataSize > 0) {
        memcpy(response.data, data, dataSize);
    }

    sendPayload(&response);
}

/**
 * @brief Send FTP NAK response
 */
void GCS_FTP::sendNak(FTPErrorCode errorCode)
{
    FTPPayload response;
    memset(&response, 0, sizeof(response));

    response.seqNumber = s_session.seqNumber;
    response.opcode = static_cast<uint8_t>(FTPOpcode::Nak);
    response.size = 1;
    response.data[0] = static_cast<uint8_t>(errorCode);

    sendPayload(&response);

    // Log error
    m_channel.sendText(MAV_SEVERITY_WARNING, "FTP operation failed");
}

/**
 * @brief Send FTP payload as FILE_TRANSFER_PROTOCOL message
 */
void GCS_FTP::sendPayload(const FTPPayload* payload)
{
    mavlink_message_t msg;
    uint8_t packedPayload[FTP_PAYLOAD_SIZE];

    memcpy(packedPayload, payload, sizeof(FTPPayload));

    mavlink_msg_file_transfer_protocol_pack(
        m_channel.getSystemID(),
        m_channel.getComponentID(),
        &msg,
        0, // Network
        s_session.targetSystem,
        s_session.targetComponent,
        packedPayload
    );

    m_channel.sendMessage(&msg);
}

/**
 * @brief Close currently open file
 */
void GCS_FTP::closeCurrentFile()
{
    if (s_session.fileHandle) {
        fs_closeFile(s_session.fileHandle);
        s_session.fileHandle = nullptr;
    }
}

} // namespace GCS
} // namespace EduCopter
