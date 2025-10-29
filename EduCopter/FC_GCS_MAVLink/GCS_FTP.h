/**
 * @file GCS_FTP.h
 * @brief EduCopter MAVLink FTP (File Transfer Protocol)
 *
 * Implements file upload/download over MAVLink using
 * FILE_TRANSFER_PROTOCOL message.
 *
 * @author EduCopter Project
 * @date 2025-10-28
 * @version 1.0.0
 */

#pragma once

#include "GCS_config.h"

#if EDUCOPTER_FTP_ENABLED

#include "GCS_MAVLink.h"
#include <stdint.h>

namespace EduCopter {
namespace GCS {

/// Maximum FTP payload size
#define EDUCOPTER_FTP_PAYLOAD_SIZE 239

/**
 * @enum FTPOpcode
 * @brief FTP operation codes
 */
enum class FTPOpcode : uint8_t {
    None = 0,
    TerminateSession = 1,
    ResetSessions = 2,
    ListDirectory = 3,
    OpenFileRO = 4,
    ReadFile = 5,
    CreateFile = 6,
    WriteFile = 7,
    RemoveFile = 8,
    CreateDirectory = 9,
    RemoveDirectory = 10,
    OpenFileWO = 11,
    TruncateFile = 12,
    Rename = 13,
    CalcFileCRC32 = 14,
    BurstReadFile = 15,
    Ack = 128,
    Nack = 129
};

/**
 * @enum FTPError
 * @brief FTP error codes
 */
enum class FTPError : uint8_t {
    None = 0,
    Fail = 1,
    FailErrno = 2,
    InvalidDataSize = 3,
    InvalidSession = 4,
    NoSessionsAvailable = 5,
    EndOfFile = 6,
    UnknownCommand = 7,
    FileExists = 8,
    FileProtected = 9,
    FileNotFound = 10
};

/**
 * @struct FTPTransaction
 * @brief Represents a single FTP request/response
 */
struct FTPTransaction {
    uint32_t offset;              ///< File offset
    uint16_t seqNumber;           ///< Sequence number
    FTPOpcode opcode;             ///< Operation code
    FTPOpcode reqOpcode;          ///< Request opcode
    uint8_t size;                 ///< Data size
    uint8_t session;              ///< Session ID
    uint8_t sysid;                ///< System ID
    uint8_t compid;               ///< Component ID
    uint8_t data[EDUCOPTER_FTP_PAYLOAD_SIZE]; ///< Payload data
};

/**
 * @class FTPSession
 * @brief Represents an active FTP session
 */
class FTPSession {
public:
    /**
     * @brief Constructor
     */
    FTPSession();

    /**
     * @brief Check if session is active
     */
    bool isActive() const { return m_active; }

    /**
     * @brief Open session
     * @param sessionID Session identifier
     * @param sysid System ID
     * @param compid Component ID
     */
    void open(uint8_t sessionID, uint8_t sysid, uint8_t compid);

    /**
     * @brief Close session
     */
    void close();

    /**
     * @brief Open file
     * @param path File path
     * @param write true for write, false for read
     * @return true if successful
     */
    bool openFile(const char* path, bool write);

    /**
     * @brief Close file
     */
    void closeFile();

    /**
     * @brief Read from file
     * @param buffer Output buffer
     * @param size Number of bytes to read
     * @return Bytes read, or -1 on error
     */
    int readFile(uint8_t* buffer, uint16_t size);

    /**
     * @brief Write to file
     * @param buffer Data to write
     * @param size Number of bytes
     * @return Bytes written, or -1 on error
     */
    int writeFile(const uint8_t* buffer, uint16_t size);

    /**
     * @brief Seek in file
     * @param offset Offset to seek to
     * @return true if successful
     */
    bool seekFile(uint32_t offset);

    // Getters
    uint8_t getSessionID() const { return m_sessionID; }
    uint8_t getSystemID() const { return m_sysid; }
    uint8_t getComponentID() const { return m_compid; }
    uint32_t getLastActivityMS() const { return m_lastActivityMS; }

    /**
     * @brief Update last activity time
     */
    void updateActivity();

private:
    bool m_active;              ///< Session is active
    uint8_t m_sessionID;        ///< Session identifier
    uint8_t m_sysid;            ///< System ID
    uint8_t m_compid;           ///< Component ID
    int m_fileHandle;           ///< File handle
    bool m_writeMode;           ///< Write mode flag
    uint32_t m_lastActivityMS;  ///< Last activity time
};

/**
 * @class GCS_FTP
 * @brief MAVLink FTP server
 */
class GCS_FTP {
public:
    /**
     * @brief Constructor
     * @param channel GCS channel for communication
     */
    explicit GCS_FTP(GCSChannel& channel);

    /**
     * @brief Destructor
     */
    ~GCS_FTP() = default;

    /**
     * @brief Initialize FTP server
     * @return true if successful
     */
    bool initialize();

    /**
     * @brief Handle FILE_TRANSFER_PROTOCOL message
     * @param msg MAVLink message
     */
    void handleFileTransferProtocol(const mavlink_message_t& msg);

    /**
     * @brief Update FTP server (process queued requests)
     */
    void update();

private:
    // ========== SESSION MANAGEMENT ==========

    /**
     * @brief Allocate a new session
     * @param sysid System ID
     * @param compid Component ID
     * @return Session ID, or 0xFF if none available
     */
    uint8_t allocateSession(uint8_t sysid, uint8_t compid);

    /**
     * @brief Get session by ID
     * @param sessionID Session identifier
     * @return Pointer to session, or nullptr if invalid
     */
    FTPSession* getSession(uint8_t sessionID);

    /**
     * @brief Terminate session
     * @param sessionID Session to terminate
     */
    void terminateSession(uint8_t sessionID);

    /**
     * @brief Reset all sessions
     */
    void resetAllSessions();

    /**
     * @brief Close timed-out sessions
     * @param timeoutMS Timeout in milliseconds
     */
    void closeTimedOutSessions(uint32_t timeoutMS = 30000);

    // ========== REQUEST HANDLERS ==========

    void handleListDirectory(FTPTransaction& request, FTPTransaction& response);
    void handleOpenFileRO(FTPTransaction& request, FTPTransaction& response);
    void handleOpenFileWO(FTPTransaction& request, FTPTransaction& response);
    void handleReadFile(FTPTransaction& request, FTPTransaction& response);
    void handleWriteFile(FTPTransaction& request, FTPTransaction& response);
    void handleRemoveFile(FTPTransaction& request, FTPTransaction& response);
    void handleCreateDirectory(FTPTransaction& request, FTPTransaction& response);
    void handleRemoveDirectory(FTPTransaction& request, FTPTransaction& response);
    void handleTruncateFile(FTPTransaction& request, FTPTransaction& response);
    void handleRename(FTPTransaction& request, FTPTransaction& response);
    void handleCalcFileCRC32(FTPTransaction& request, FTPTransaction& response);
    void handleBurstReadFile(FTPTransaction& request, FTPTransaction& response);

    // ========== HELPERS ==========

    /**
     * @brief Send response
     * @param response Response transaction
     * @return true if sent
     */
    bool sendResponse(const FTPTransaction& response);

    /**
     * @brief Send error response
     * @param request Original request
     * @param error Error code
     */
    void sendError(const FTPTransaction& request, FTPError error);

    /**
     * @brief Validate path (security check)
     * @param path Path to validate
     * @return true if safe
     */
    bool validatePath(const char* path);

    // ========== MEMBER VARIABLES ==========

    FTPSession m_sessions[EDUCOPTER_MAX_FTP_SESSIONS]; ///< Active sessions
    bool m_initialized;                                 ///< Initialized flag
};

} // namespace GCS
} // namespace EduCopter

#endif // EDUCOPTER_FTP_ENABLED
