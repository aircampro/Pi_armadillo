/*
 * Matching regular expressions by boost.
 */
#include <iostream>
#include <string>
#include <boost/regex.hpp>     // require "boost-regex-dev"

using namespace std;

/*
 * [CLASS] Process
 */
class Proc
{
    // Private Declaration
    const char    *cSrc, *cPtnA, *cPtnS; // Source string, Regex pattern (char*)
    string        sSrc, sPtn;            // Source string, Regex pattern (string)
    boost::regex  reA, reS;              // Regular expression
    boost::cmatch cm;                    // Match result (char*)
    boost::smatch sm;                    // Match result (string)
    string::const_iterator start, end;   // Iterator
    bool regexCharAll();                 // All matches (char*)
    bool regexStringAll();               // All matches (string)
    bool regexRangeAll();                // All matches (range)
    bool regexCharSub();                 // Sub matches (char*)
    bool regexStringSub();               // Sub matches (string)
    bool regexRangeSub();                // Sub matches (range)

public:
    Proc();           // Constructor
    bool execMain();  // Main Process
};

/*
 * Proc - Constructor
 */
Proc::Proc()
{
    // Initial settings
    cSrc  = "This is a test of irregular expressions.";
    cPtnA = "(.*)\\s*(regular)\\s*(.*)";
    cPtnS = "re(.*)ar";
    sSrc  = cSrc;
    reA   = cPtnA;
    reS   = cPtnS;
    cout << "[Source string     ] " << cSrc  << "\n"
         << "[Regex pattern(All)] " << cPtnA << "\n"
         << "[Regex pattern(Sub)] " << cPtnS << "\n"
         << endl;
}

/*
 * Main Process
 */
bool Proc::execMain()
{
    try {
        // All mathces (char*)
        if (!regexCharAll())   return false;

        // All mathces (string)
        if (!regexStringAll()) return false;

        // All mathces (range)
        if (!regexRangeAll())  return false;

        // Sub mathces (char*)
        if (!regexCharSub())   return false;

        // Sub mathces (string)
        if (!regexStringSub()) return false;

        // Sub mathces (range)
        if (!regexRangeSub())  return false;
    } catch (char *e) {
        cerr << "[EXCEPTION] " << e << endl;
        return false;
    }
    return true;
}

// All matches (char*)
bool Proc::regexCharAll()
{
    cout << "* All match - char*\n";
    try {
        if (boost::regex_match(cSrc, cm, reA)) {
            cout << "  ==== Matched ====\n";
            for (size_t i = 0; i < cm.size(); ++i) {
                cout << "  [" << i << "] (pos = " << cm.position(i) << ", "
                     << "len = " << cm.length(i) << ") "
                     << cm.str(i) << "\n";
            }
        } else {
            cout << "  ==== Unmatched ====\n";
        }
    } catch (char *e) {
        cerr << "[EXCEPTION] " << e << endl;
        return false;
    }
    cout << endl;
    return true;
}

// All matches (string)
bool Proc::regexStringAll()
{
    cout << "* All match - string\n";
    try {
        if (boost::regex_match(sSrc, sm, reA)) {
            cout << "  ==== Matched ====\n";
            for (size_t i = 0; i < sm.size(); ++i) {
                cout << "  [" << i << "] (pos = " << sm.position(i) << ", "
                     << "len = " << sm.length(i) << ") "
                     << sm.str(i) << "\n";
            }
        } else {
            cout << "  ==== Unmatched ====\n";
        }
    } catch (char *e) {
        cerr << "[EXCEPTION] " << e << endl;
        return false;
    }
    cout << endl;
    return true;
}

// All matches (range)
bool Proc::regexRangeAll()
{
    cout << "* All match - range\n";
    try {
        start = sSrc.begin();
        end   = sSrc.end();
        if (boost::regex_match(start, end, sm, reA)) {
            cout << "  ==== Matched ====\n";
            for (size_t i = 0; i < sm.size(); ++i) {
                cout << "  [" << i << "] (pos = " << sm.position(i) << ", "
                     << "len = " << sm.length(i) << ") "
                     << sm.str(i) << "\n";
            }
        } else {
            cout << "  ==== Unmatched ====\n";
        }
    } catch (char *e) {
        cerr << "[EXCEPTION] " << e << endl;
        return false;
    }
    cout << endl;
    return true;
}

// Sub matches (char*)
bool Proc::regexCharSub()
{
    cout << "* Sub match - char*\n";
    try {
        if (boost::regex_search(cSrc, cm, reS)) {
            cout << "  ==== Matched ====\n";
            for (size_t i = 0; i < cm.size(); ++i) {
                cout << "  [" << i << "] (pos = " << cm.position(i) << ", "
                     << "len = " << cm.length(i) << ") "
                     << cm.str(i) << "\n";
            }
        } else {
            cout << "  ==== Unmatched ====\n";
        }
    } catch (char *e) {
        cerr << "[EXCEPTION] " << e << endl;
        return false;
    }
    cout << endl;
    return true;
}

// Sub matches (string)
bool Proc::regexStringSub()
{
    cout << "* Sub match - string\n";
    try {
        if (boost::regex_search(sSrc, sm, reS)) {
            cout << "  ==== Matched ====\n";
            for (size_t i = 0; i < sm.size(); ++i) {
                cout << "  [" << i << "] (pos = " << sm.position(i) << ", "
                     << "len = " << sm.length(i) << ") "
                     << sm.str(i) << "\n";
            }
        } else {
            cout << "  ==== Unmatched ====\n";
        }
    } catch (char *e) {
        cerr << "[EXCEPTION] " << e << endl;
        return false;
    }
    cout << endl;
    return true;
}

// Sub matches (range)
bool Proc::regexRangeSub()
{
    cout << "* Sub match - range\n";
    try {
        start = sSrc.begin();
        end   = sSrc.end();
        if (boost::regex_search(start, end, sm, reS)) {
            cout << "  ==== Matched ====\n";
            for (size_t i = 0; i < sm.size(); ++i) {
                cout << "  [" << i << "] (pos = " << sm.position(i) << ", "
                     << "len = " << sm.length(i) << ") "
                     << sm.str(i) << "\n";
            }
        } else {
            cout << "  ==== Unmatched ====\n";
        }
    } catch (char *e) {
        cerr << "[EXCEPTION] " << e << endl;
        return false;
    }
    cout << endl;
    return true;
}

/*
 * Execution
 */
int main(){
    try {
        Proc objMain;
        bool bRet = objMain.execMain();
        if (!bRet) cout << "ERROR!" << endl;
    } catch (char *e) {
        cerr << "[EXCEPTION] " << e << endl;
        return 1;
    }
    return 0;
}
