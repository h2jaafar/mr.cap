#include <iostream>
#include <deque>
#include <string>

enum LogLevel {
    LOG_INFO,
    LOG_WARNING,
    LOG_ERROR,
    LOG_DEBUG
};

struct LogEntry {
    LogLevel level;
    std::string message;
};

class ImGuiLogger {
public:
    void AddLog(LogLevel level, const char* fmt, ...) {
        va_list args1;
        va_list args2;
        va_start(args1, fmt);
        va_copy(args2, args1); // create a copy of va_list for the second vsnprintf call

        LogEntry entry;
        entry.level = level;

        size_t message_size = std::vsnprintf(nullptr, 0, fmt, args1) + 1; // use args1
        va_end(args1); // end args1

        entry.message.resize(message_size - 1); // Make space for the null character

        std::vsnprintf(&entry.message[0], message_size, fmt, args2); // use args2
        va_end(args2); // end args2

        m_logEntries.push_back(entry);
    }



    void Draw(const char* title) {
        ImGui::Begin(title);

        // Search bar
        ImGui::InputText("Search", m_searchBuffer, 256);

        // Checkboxes for log levels
        ImGui::Checkbox("Info", &m_showInfo);
        ImGui::SameLine();
        ImGui::Checkbox("Warning", &m_showWarning);
        ImGui::SameLine();
        ImGui::Checkbox("Error", &m_showError);
        ImGui::SameLine();
        ImGui::Checkbox("Debug", &m_showDebug);

        // Clear button
        if (ImGui::Button("Clear")) {
            m_logEntries.clear();
        }

        ImGui::BeginChild("LogEntries");

        // Iterate backwards through log entries so that we display the most recent entries first
        for (auto it = m_logEntries.rbegin(); it != m_logEntries.rend(); ++it) {
            // Check if the log entry should be displayed based on the log level
            if (ShouldDisplay((*it).level)) {
                // Check if the log entry should be displayed based on the search term
                bool matchesSearch = m_searchBuffer[0] == '\0' || strstr((*it).message.c_str(), m_searchBuffer) != nullptr;

                if (matchesSearch) {
                    ImVec4 color = GetLevelColor((*it).level);
                    ImGui::TextColored(color, "%s", (*it).message.c_str());
                }
            }
        }

        ImGui::EndChild();
        ImGui::End();
    }

private:
    bool ShouldDisplay(LogLevel level) const {
        switch (level) {
            case LOG_INFO: return m_showInfo;
            case LOG_WARNING: return m_showWarning;
            case LOG_ERROR: return m_showError;
            case LOG_DEBUG: return m_showDebug;
        }
        return false;
    }

    ImVec4 GetLevelColor(LogLevel level) const {
        switch (level) {
            case LOG_INFO: return ImVec4(1.0f, 1.0f, 1.0f, 1.0f);   // White color
            case LOG_WARNING: return ImVec4(0.0f, 1.0f, 0.0f, 1.0f); // Green color
            case LOG_ERROR: return ImVec4(1.0f, 0.0f, 0.0f, 1.0f);   // Red color
            case LOG_DEBUG: return ImVec4(0.5f, 0.5f, 0.5f, 1.0f);   // Unchanged color
        }
        return ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
    }

    std::deque<LogEntry> m_logEntries;
    bool m_showInfo = true;
    bool m_showWarning = true;
    bool m_showError = true;
    bool m_showDebug = false;
    char m_searchBuffer[256] = "";
};

ImGuiLogger logger;