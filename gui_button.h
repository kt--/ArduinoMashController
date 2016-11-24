////
//// Copyright (C) 2016 Kingsley Turner, krt@krt.com.au
//// See LICENCE.txt for details, but licensed CC BY-NC-SA
////

#ifndef __GUI_BUTTON_H__
#define __GUI_BUTTON_H__

/*
#include <SPI.h>        // must include this here (or else IDE can't find it)
#include <PDQ_GFX.h>             // PDQ: Core graphics library
#define ILI9341_CS_PIN      10   // <= /CS pin (chip-select, LOW to get attention of ILI9341, HIGH and it ignores SPI bus)
#define ILI9341_DC_PIN       9   // <= DC pin (1=data or 0=command indicator line) also called RS
#define ILI9341_RST_PIN      8   // <= RST pin (optional)
#define ILI9341_SAVE_SPCR    0   // <= 0/1 with 1 to save/restore AVR SPI control register (to "play nice" when other SPI use)
#include <PDQ_ILI9341.h>         // PDQ: Hardware-specific driver library
*/

// Turn this on to allow setting the colours dynamically
// but turning this on uses about a kilobyte (or more) of memory.
// Seems a lot for what it is, but there-ya-go.
#undef ALLOW_COLOUR_SETTING


#define COLOUR_BASE              0x000d  // Dark Blue
#define COLOUR_TEXT              0xfe60  // golden
#define COLOUR_BORDER            0xfe60
#define COLOUR_BASE_SELECTED     0xb085  // Crimson
#define COLOUR_TEXT_SELECTED     0xffff
#define COLOUR_BORDER_SELECTED   0xffff



#include "utils.h"

// Base font size is 6x7, using text-size=2 ->
#define FONT_TEXT_WIDTH  12
#define FONT_TEXT_HEIGHT 14

#define MENU_SELECTION_NONE      0
#define MENU_SELECTION_HIGHLIGHT 1
#define MENU_SELECTION_ACTIVE    2

// Three types of menu-item/edit-field
#define MENU_ENTRY_STRING 0
#define MENU_ENTRY_FLOAT  1
#define MENU_ENTRY_LONG   2



void resetTFT();
template<class HW> void printTFT(PDQ_GFX<HW> *screen, int cursor_x, int cursor_y, int text_size, const char *str);
template<class HW> void printNumberTFT(PDQ_GFX<HW> *screen, int cursor_x, int cursor_y, int text_size, int v,   int width=0, char pad='0');
template<class HW> void printNumberTFT(PDQ_GFX<HW> *screen, int cursor_x, int cursor_y, int text_size, float v, int decimal_places=2);

////
//// NOTE: My apologies, but all of the error checking has been removed to save space.
////

// A menu stop is a single entry in a menu.
class GUIMenuStop
{
public:
    void setAll(const void *l, unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int places)
    {
        pos_x = x;
        pos_y = y;
        width = w;
        height= h;
        label = (const char *)l;  // all types in the union are pointers
        decimal_places = places;
    }
    
    void set(const char *str, unsigned int x, unsigned int y, unsigned int w, unsigned int h)
    {
        setAll(str, x, y, w, h, 0);
        mode = MENU_ENTRY_STRING;      
    }
    
    void set(float *v, int places, unsigned int x, unsigned int y, unsigned int w, unsigned int h)
    {
        setAll(v, x, y, w, h, places);
        mode = MENU_ENTRY_FLOAT;      
    }

    void set(long *l, int places, unsigned int x, unsigned int y, unsigned int w, unsigned int h)
    {
        setAll(l, x, y, w, h, places);
        mode = MENU_ENTRY_LONG;      
    }

    
#ifdef ALLOW_COLOUR_SETTING    
    template<class HW>
    void paint(PDQ_GFX<HW> *screen, int highlight_mode, 
               unsigned int base_colour,          unsigned int text_colour,          unsigned int border_colour, 
               unsigned int selected_base_colour, unsigned int selected_text_colour, unsigned int selected_border_colour)
#else
    template<class HW>
    void paint(PDQ_GFX<HW> *screen, int highlight_mode)
#endif
    {
        /*
        // getTextBounds() does not seem to work
        int16_t x1, y1;
        unsigned int wid, hei;
        screen->getTextBounds(label, pos_x, pos_y, &x1, &y1, &wid, &hei);
        */
       
#ifndef ALLOW_COLOUR_SETTING
        const unsigned int base_colour   = COLOUR_BASE;
        const unsigned int text_colour   = COLOUR_TEXT;
        const unsigned int border_colour = COLOUR_BORDER;
        const unsigned int selected_base_colour = COLOUR_BASE_SELECTED;
        const unsigned int selected_text_colour = COLOUR_TEXT_SELECTED;
        const unsigned int selected_border_colour = COLOUR_BORDER_SELECTED;
#endif
       
        if (highlight_mode == MENU_SELECTION_HIGHLIGHT)
        {
            // Selected button is inverse with border
            screen->fillRect_(pos_x, pos_y, width, height, text_colour);
            if (border_colour != base_colour)
                screen->drawRect(pos_x, pos_y, width, height, border_colour);
            screen->setTextColor(base_colour);
        }        
        else if (highlight_mode == MENU_SELECTION_ACTIVE)
        {
            // Selected IN-USE button is red & white
            screen->fillRect_(pos_x, pos_y, width, height, selected_base_colour);
            if (selected_border_colour != selected_base_colour)
            {
                screen->drawRect(pos_x, pos_y, width, height, selected_border_colour);
                screen->drawRect(pos_x+1, pos_y+1, width-2, height-2, selected_border_colour);  // Make border thicker - not sure why this needs to be -2 (and not -1).
            }
            screen->setTextColor(selected_text_colour);
        }
        else
        {
            screen->fillRect_(pos_x, pos_y, width, height, base_colour);
            screen->setTextColor(text_colour);
        }


        const char *final_str = label;
        char num_buffer[16];
        if (mode == MENU_ENTRY_FLOAT)
        {
            ftoa(*float_value, num_buffer, decimal_places);
            /*
            if (decimal_places == 0)
            {
                 // Erase the decimals
                 char *ptr = num_buffer;
                 while (*ptr != '\0')
                 {
                     if (*ptr == '.')
                         *ptr = '\0';
                     else
                         ptr++;
                 }
            }
            */
            final_str = num_buffer;
        }
        else if (mode == MENU_ENTRY_LONG)
        {
            itoa(*long_value, num_buffer, 10);
            final_str = num_buffer;
        }

        // Estimate the text width
        unsigned int text_width = strlen(final_str) * FONT_TEXT_WIDTH;

        // if the text can fit within the button, centre it.
        unsigned int text_x = pos_x;
        unsigned int text_y = pos_y;
        if (text_width < width)
        {
            text_x = 1+ pos_x + (width - text_width) / 2;
            text_y = pos_y + (height - FONT_TEXT_HEIGHT) / 2;  // Assumes only 1 line of text
        }

        printTFT(screen, text_x, text_y, 2, final_str);
    }    


    void setValue(float *v)      { float_value = v;    }
    void setValue(long *l)       { long_value = l;    }
    float getValue()             
    { 
        float rc = *float_value;
        if (mode == MENU_ENTRY_LONG)
            rc = (float) *long_value; 
        return rc;
    }
    
    void setLabel(const char *l) { label = l;    }
    const char *getLabel()       { return label; }

private:    
    union
    {
        float *float_value;
        long  *long_value;
        const char *label;    
    };

    unsigned char pos_x;
    unsigned char pos_y;
    unsigned char width;
    unsigned char height;
    // Note: Had these stored as mode:4 and decimal_places:4, but it took more RAM, not less.
    unsigned char mode;   
    unsigned char decimal_places;

};



// A GUIMenu is just a bunch of GUIMenuStops.
class GUIMenu
{
public:
    GUIMenu(unsigned char entry_count)
    {
        menu_entries = new GUIMenuStop[entry_count];
        menu_entries_count = entry_count;
        selected_entry = 0;
        selected_mode  = MENU_SELECTION_HIGHLIGHT;
    }    

    /* This will never be called
    ~GUIMenu()
    {
        if (menu_entries != NULL)
        {
            if (menu_entries != NULL)
                delete[] menu_entries;
        }
    }
    */
   
    void addMenuEntry(unsigned int index,const char *label, unsigned int x, unsigned int y, unsigned int w, unsigned int h)
    {
        menu_entries[index].set(label, x, y, w, h);
    }
    void addMenuEntry(unsigned int index, float *value, int decimal_places, unsigned int x, unsigned int y, unsigned int w, unsigned int h)
    {
        menu_entries[index].set(value, decimal_places, x, y, w, h);
    }
    void addMenuEntry(unsigned int index, long *value, int decimal_places, unsigned int x, unsigned int y, unsigned int w, unsigned int h)
    {
        menu_entries[index].set(value, decimal_places, x, y, w, h);
    }


    void setHighlightedEntry(unsigned char index)
    {
        selected_entry = index;
        selected_mode  = MENU_SELECTION_HIGHLIGHT;
    }
    void setActivatedEntry(unsigned char index)
    {
        selected_entry = index;
        selected_mode  = MENU_SELECTION_ACTIVE;
    }
       
    template<class HW> 
    void paint(PDQ_GFX<HW> *screen)
    {
        unsigned char entry_mode;
        for (int i=0; i<menu_entries_count; i++)
        {
            if (selected_entry == i)
                entry_mode = selected_mode;
            else
                entry_mode = MENU_SELECTION_NONE;
                
#ifdef ALLOW_COLOUR_SETTING                
            menu_entries[i].paint(screen, entry_mode, base_colour, text_colour, border_colour, selected_base_colour, selected_text_colour, selected_border_colour);
#else
            menu_entries[i].paint(screen, entry_mode);
#endif
        }
    }    
  
    template<class HW>
    void paint(PDQ_GFX<HW> *screen, int index)
    {
        unsigned char entry_mode = MENU_SELECTION_NONE;
        if (selected_entry == index)
            entry_mode = selected_mode;
    
#ifdef ALLOW_COLOUR_SETTING    
        menu_entries[index].paint(screen, entry_mode, base_colour, text_colour, border_colour, selected_base_colour, selected_text_colour, selected_border_colour);
#else
        menu_entries[index].paint(screen, entry_mode);
#endif
    }    

#ifdef ALLOW_COLOUR_SETTING    
    void setColours(unsigned int bc, unsigned int tc, unsigned int bdc, unsigned int sbc, unsigned int stc, unsigned int sbdc)
    {
        base_colour            = bc;
        text_colour            = tc;
        border_colour          = bdc;
        selected_base_colour   = sbc;
        selected_text_colour   = stc;
        selected_border_colour = sbdc;
    }   
#else
    void setColours(unsigned int bc, unsigned int tc, unsigned int bdc, unsigned int sbc, unsigned int stc, unsigned int sbdc)
    {
        // hopefully the compiler optimises this out
    }
#endif
    
    void getCurrentSelection(int *item_number, int *item_mode)
    {
        *item_number = selected_entry;
        *item_mode   = selected_mode;
    }

    
    void setMenuEntryValue(int index, float *v)
    {
         menu_entries[index].setValue(v);        
    }
    void setMenuEntryValue(int index, long *l)
    {
         menu_entries[index].setValue(l);        
    }
    float getMenuEntryValue(int index)
    {
        return menu_entries[index].getValue();
    }
    
    void highlightCurrentEntry() { selected_mode = MENU_SELECTION_HIGHLIGHT; }  // changes the way a button is painted too
    void activateCurrentEntry()  { selected_mode = MENU_SELECTION_ACTIVE; }    
    void setFocusedEntry(int e)  { selected_entry = e; }
    const char *getMenuEntryLabel(int index) 
    {
        return menu_entries[index].getLabel();
    }
    void setMenuEntryLabel(int index, const char *l) 
    {
        menu_entries[index].setLabel(l);
    }
        
        
    // Highlight a button for 1/2 a second before de-highlighting it
    // this is used to flash a menu-exit button, etc.
    template<class HW>
    void flashAndDeactivateCurrentEntry(PDQ_GFX<HW> *screen)
    {
        selected_mode = MENU_SELECTION_ACTIVE;
        paint(screen);
        delay(500);
        selected_mode = MENU_SELECTION_HIGHLIGHT; 
        paint(screen);       
    }
    
private:
    GUIMenuStop *menu_entries;

#ifdef ALLOW_COLOUR_SETTING
    unsigned int base_colour;
    unsigned int text_colour;
    unsigned int border_colour;
    unsigned int selected_base_colour;
    unsigned int selected_text_colour;
    unsigned int selected_border_colour;
#endif    
    
    unsigned char menu_entries_count;
    unsigned char selected_entry;
    unsigned char selected_mode;
};





void resetTFT()
{
#if defined(ILI9341_RST_PIN)  // reset like Adafruit does
    FastPin<ILI9341_RST_PIN>::setOutput();
    FastPin<ILI9341_RST_PIN>::hi();
    FastPin<ILI9341_RST_PIN>::lo();
    delay(1);
    FastPin<ILI9341_RST_PIN>::hi();
#endif  
}

template<class HW>
void printTFT(PDQ_GFX<HW> *screen, int cursor_x, int cursor_y, int text_size, const char *str)
{
   screen->setTextSize(text_size);
   screen->setCursor(cursor_x, cursor_y);
   screen->print(str);
}


template<class HW>
void printNumberTFT(PDQ_GFX<HW> *screen, int cursor_x, int cursor_y, int text_size, int v, int width/*=0*/, char pad/*='0'*/)
{
   
   char  str[16] = { 0 };
   itoa(v, str, 10);
   if (width > 0 && width < sizeof(str)-1)
   {
       unsigned int len = strlen(str);
       if (width > len)
       {
           for (unsigned int i=0; i<(width-len); i++)
               str[i]=pad;
           str[width]='\0';
       }
   }
   printTFT(screen, cursor_x, cursor_y, text_size, str);
}


template<class HW>
void printNumberTFT(PDQ_GFX<HW> *screen, int cursor_x, int cursor_y, int text_size, float v, int decimal_places)
{
   char  str[16] = { 0 };
   ftoa(v, str, decimal_places);
   printTFT(screen, cursor_x, cursor_y, text_size, str);
}




#endif //#ifndef __GUI_BUTTON_H__
