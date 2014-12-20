#ifndef _TEXTHELPER_
#define _TEXTHELPER_
class TextHelper
{
public:

    inline static bool ReadLine(char *&buffer, std::string &line)
    {
        ParseBlank(buffer);

        char *p = buffer;
        while (*p != 0 && *p != '\n')
            p++;

        int length = p - buffer;
        line = string(buffer, length);//todo trim
        buffer = p;
        return true;
    }

    inline static bool ParseToken(char *&buffer, string &token)
    {
        ParseBlank(buffer);

        char *p = buffer;
        while (*p != 0 && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n')
            p++;

        int length = p - buffer;
        if (length > 0)
        {
            token = string(buffer, length);
            buffer = p;
            return true;
        }
        return false;
    }

    inline static bool ParseInteger(char *&buffer, int &value)
    {
        char *p = buffer;
        bool is_negative = false;
        if (*p == '+')
        {
            p++;
        }
        else if (*p == '-')
        {
            p++;
            is_negative = true;
        }
        int v = 0;
        while ((*p >= '0') && (*p <= '9'))
        {
            v = v * 10 + (*p - '0');
            p++;
        }
        if (is_negative)
            value = -v;
        else
            value = v;
        buffer = p;
        return true;
    }

    // The same code as inline static bool ParseInteger(char *&buffer, int &value)
    inline static bool Parse(char *&buffer, int &value)
    {
        char *p = buffer;
        bool is_negative = false;
        if (*p == '+')
        {
            p++;
        }
        else if (*p == '-')
        {
            p++;
            is_negative = true;
        }
        int v = 0;
        while ((*p >= '0') && (*p <= '9'))
        {
            v = v * 10 + (*p - '0');
            p++;
        }
        if (is_negative)
            value = -v;
        else
            value = v;
        buffer = p;
        return true;
    }

    template <typename Type>
    inline static bool Parse(char *&buffer, Type &value)
    {
        int integer_value = 0;
        // do not check whether integer_value is in [minimum(unsigned char), maximum(unsigned char)],
        // for performance considerations
        if (ParseInteger(buffer, integer_value))
        {
            value = (Type)integer_value;
            return true;
        }
        return false;
    }

    inline static bool ParseFloat(char *&buffer, float &value)
    {
        char *p = buffer;
        bool is_negative = false;
        if (*p == '+')
        {
            p++;
        }
        else if (*p == '-')
        {
            p++;
            is_negative = true;
        }
        float v = 0.0f;
        while ((*p >= '0') && (*p <= '9'))
        {
            v = v * 10 + (*p - '0');
            p++;
        }
        if (*p == '.')
        {
            p++;
            float w = 0.1f;
            while ((*p >= '0') && (*p <= '9'))
            {
                v += w * (*p - '0');
                w *= 0.1f;
                p++;
            }
        }
        if ((*p == 'e') || (*p == 'E'))
        {
            p++;
            int exp = 0;
            if (ParseInteger(p, exp))
                v *= pow(10.0f, float(exp));
        }
        buffer = p;
        if (is_negative)
            value = -v;
        else
            value = v;
        return true;
    }

    // same code as inline static bool ParseFloat(char *&buffer, float &value)
    inline static bool Parse(char *&buffer, float &value)
    {
        char *p = buffer;
        bool is_negative = false;
        if (*p == '+')
        {
            p++;
        }
        else if (*p == '-')
        {
            p++;
            is_negative = true;
        }
        float v = 0.0f;
        while ((*p >= '0') && (*p <= '9'))
        {
            v = v * 10 + (*p - '0');
            p++;
        }
        if (*p == '.')
        {
            p++;
            float w = 0.1f;
            while ((*p >= '0') && (*p <= '9'))
            {
                v += w * (*p - '0');
                w *= 0.1f;
                p++;
            }
        }
        if ((*p == 'e') || (*p == 'E'))
        {
            p++;
            int exp = 0;
            if (ParseInteger(p, exp))
                v *= pow(10.0f, float(exp));
        }
        buffer = p;
        if (is_negative)
            value = -v;
        else
            value = v;
        return true;
    }

    // same code as inline static bool ParseReal(char *&buffer, System::real &value)
    inline static bool Parse(char *&buffer, double &value)
    {
        char *p = buffer;
        bool is_negative = false;
        if (*p == '+')
        {
            p++;
        }
        else if (*p == '-')
        {
            p++;
            is_negative = true;
        }
        double v = 0;
        while ((*p >= '0') && (*p <= '9'))
        {
            v = v * 10 + (*p - '0');
            p++;
        }
        if (*p == '.')
        {
            p++;
            double w = 0.1;
            while ((*p >= '0') && (*p <= '9'))
            {
                v += w * (*p - '0');
                w *= 0.1;
                p++;
            }
        }
        if ((*p == 'e') || (*p == 'E'))
        {
            p++;
            int exp = 0;
            if (ParseInteger(p, exp))
                v *= pow(10.0, exp);
        }
        buffer = p;
        if (is_negative)
            value = -v;
        else
            value = v;
        return true;
    }

    inline static bool ParseBlank(char *&buffer)
    {
        char *p = buffer;
        while (true)
        {
            char c = *p;
            if ((c == ' ') || (c == '\t'))
                p++;
            else if (c == 0)
                return false;
            else
                break;
        }
        buffer = p;
        return true;
    }

    inline static bool ParseBlankNewline(char *&buffer)
    {
        char *p = buffer;
        while (true)
        {
            char c = *p;
            if ((c == ' ') || (c == '\t') || (c == '\r') || (c == '\n'))
                p++;
            else if (c == 0)
                return false;
            else
                break;
        }
        buffer = p;
        return true;
    }

    inline static bool ParseSpecifiedSequence(char *&buffer, const char *sequence, int length)
    {
        for (int i = 0; i < length; i++)
        {
            if (*buffer != *sequence)
                return false;
            buffer++;
            sequence++;
        }
        return true;
    }

    inline static bool GotoNextLine(char *&buffer)
    {
        char *p = buffer;
        while (true)
        {
            char c = *p;
            if (c == '\n')
            {
                p++;
                break;
            }
            else if (c == 0)
            {
                return false;
            }
            else
            {
                p++;
            }
        }
        buffer = p;
        return true;
    }



};
#endif