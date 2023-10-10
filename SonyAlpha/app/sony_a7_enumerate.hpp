#ifndef __sony_enum_
#define __sony_enum_

#define SONY_ENUM_INVALID 99

std::uint32_t enumerate_still_cap_sony_a7(std::uint32_t num) {

    std::uint32_t enum_num = 0u;

    switch (num)
    {
        case 65543:
        enum_num = 2;
        break;

        case 1:
        enum_num = 0;
        break;

        case 65540:
        enum_num = 1;
        break;

        case 65537:
        enum_num = 3;
        break;

        case 65538:
        enum_num = 4;
        break;

        case 196611:
        enum_num = 5;
        break;

        case 196610:
        enum_num = 6;
        break;

        case 196609:
        enum_num = 7;
        break;

        case 524289:
        enum_num = 8;
        break;

        case 524290:
        enum_num = 9;
        break;

        case 524293:
        enum_num = 10;
        break;

        case 524294:
        enum_num = 11;
        break;

        case 524291:
        enum_num = 12;
        break;

        case 524292:
        enum_num = 13;
        break;

        case 393218:
        enum_num = 46;
        break;

        case 393217:
        enum_num = 47;
        break;

        case 458754:
        enum_num = 48;
        break;

        case 458753:
        enum_num = 49;
        break;

        default:
        if ((num >= 262913) && (num <= 262928))
            enum_num = 14 + (num - 262913);
        else if ((num >= 327681) && (num <= 327696))
            enum_num = 30 + (num - 327681);
        else
            enum_num = SONY_ENUM_INVALID;
        break;
    }
    return enum_num;
}

std::uint16_t  enumerate_aperture_sony_a7(std::uint32_t  num) {

    std::uint16_t enum_num = 0u;

    switch (num)
    {

        case 250:
        enum_num = 0;
        break;

        case 280:
        enum_num = 1;
        break;

        case 320:
        enum_num = 2;
        break;

        case 350:
        enum_num = 3;
        break;

        case 400:
        enum_num = 4;
        break;

        case 450:
        enum_num = 5;
        break;

        case 500:
        enum_num = 6;
        break;

        case 560:
        enum_num = 7;
        break;

        case 630:
        enum_num = 8;
        break;

        case 710:
        enum_num = 9;
        break;

        case 800:
        enum_num = 10;
        break;

        case 900:
        enum_num = 11;
        break;

        case 1000:
        enum_num = 12;
        break;

        case 1100:
        enum_num = 13;
        break;

        case 1300:
        enum_num = 14;
        break;

        case 1400:
        enum_num = 15;
        break;

        case 1600:
        enum_num = 16;
        break;

        case 1800:
        enum_num = 17;
        break;

        case 2000:
        enum_num = 18;
        break;

        case 2200:
        enum_num = 19;
        break;

        default:
        enum_num = SONY_ENUM_INVALID;
        break;
    }
    return enum_num;
}

std::uint32_t  enumerate_iso_sony_a7(std::uint32_t  num) {

    std::uint32_t enum_num = 0;
    switch (num)
    {
        case 0:
        enum_num = 0;
        break;

        case 50:
        enum_num = 1;
        break;

        case 64:
        enum_num = 2;
        break;

        case 80:
        enum_num = 3;
        break;

        case 100:
        enum_num = 4;
        break;

        case 125:
        enum_num = 5;
        break;

        case 160:
        enum_num = 6;
        break;

        case 200:
        enum_num = 7;
        break;

        case 250:
        enum_num = 8;
        break;

        case 320:
        enum_num = 9;
        break;

        case 400:
        enum_num = 10;
        break;

        case 500:
        enum_num = 11;
        break;

        case 640:
        enum_num = 12;
        break;

        case 800:
        enum_num = 13;
        break;

        case 1000:
        enum_num = 14;
        break;

        case 1250:
        enum_num = 15;
        break;

        case 1600:
        enum_num = 16;
        break;

        case 2000:
        enum_num = 17;
        break;

        case 2500:
        enum_num = 18;
        break;

        case 3200:
        enum_num = 19;
        break;

        case 4000:
        enum_num = 20;
        break;

        case 5000:
        enum_num = 21;
        break;

        case 6400:
        enum_num = 22;
        break;

        case 8000:
        enum_num = 23;
        break;

        case 10000:
        enum_num = 24;
        break;

        case 12800:
        enum_num = 25;
        break;

        case 16000:
        enum_num = 26;
        break;

        case 20000:
        enum_num = 27;
        break;

        case 25600:
        enum_num = 28;
        break;

        case 32000:
        enum_num = 29;
        break;

        case 40000:
        enum_num = 30;
        break;

        case 51200:
        enum_num = 31;
        break;

        case 64000:
        enum_num = 32;
        break;

        case 80000:
        enum_num = 33;
        break;

        case 102400:
        enum_num = 34;
        break;

        default:
        enum_num = SONY_ENUM_INVALID;
        break;
    }
    return enum_num;
}

std::uint16_t enumerate_ex_pro_sony_a7(std::uint32_t num) {

    std::uint32_t enum_num = 0;
    switch (num)
    {
        case 32850:
        enum_num = 2;
        break;

        case 32848:
        enum_num = 0;
        break;

        case 32849:
        enum_num = 1;
        break;

        case 32851:
        enum_num = 3;
        break;

        default:
        enum_num = SONY_ENUM_INVALID;
        break;
    }
    return enum_num;
}

std::uint16_t enumerate_focus_area_sony_a7(std::uint32_t num) {

    std::uint16_t enum_num = 0;
    if ((num >= 1) && (num <= 7))
        enum_num = num - 1;
    else
        enum_num = SONY_ENUM_INVALID;
    return enum_num;
}

std::uint16_t enumerate_focus_sony_a7(std::uint32_t num) {

    std::uint16_t enum_num = 0;

    switch (num)
    {
        case 2:
        enum_num = 0;
        break;

        case 4:
        enum_num = 1;
        break;

        case 3:
        enum_num = 2;
        break;

        case 6:
        enum_num = 3;
        break;

        case 1:
        enum_num = 4;
        break;

        default:
        enum_num = SONY_ENUM_INVALID;
        break;
    }
    return enum_num;
}

std::uint32_t enumerate_shutter_sony_a7(std::uint32_t num) {

    std::uint32_t enum_num = 0;

    //this occurs under certain conditions
    //then they all shift down by one 
    //
    switch (num)
    {
        case 0:
        enum_num = 0;
        break;

        case 19660810:
        enum_num = 1;
        break;

        case  16384010:
        enum_num = 2;
        break;

        case  13107210:
        enum_num = 3;
        break;

        case  9830410:
        enum_num = 4;
        break;

        case  8519690:
        enum_num = 5;
        break;

        case  6553610:
        enum_num = 6;
        break;

        case 5242890:
        enum_num = 7;
        break;

        case 3932170:
        enum_num = 8;
        break;

        case 3276810:
        enum_num = 9;
        break;

        case 2621450:
        enum_num = 10;
        break;

        case 2097162:
        enum_num = 11;
        break;

        case 1638410:
        enum_num = 12;
        break;

        case 1310730:
        enum_num = 13;
        break;

        case 1048586:
        enum_num = 14;
        break;

        case 851978:
        enum_num = 15;
        break;

        case 655370:
        enum_num = 16;
        break;

        case 524298:
        enum_num = 17;
        break;

        case 393226:
        enum_num = 18;
        break;

        case 327690:
        enum_num = 19;
        break;

        case 262154:
        enum_num = 20;
        break;

        case 65539:
        enum_num = 21;
        break;

        case 65540:
        enum_num = 22;
        break;

        case 65541:
        enum_num = 23;
        break;

        case 65542:
        enum_num = 24;
        break;

        case 65544:
        enum_num = 25;
        break;

        case 65546:
        enum_num = 26;
        break;

        case 65549:
        enum_num = 27;
        break;

        case 65551:
        enum_num = 28;
        break;

        case 65556:
        enum_num = 29;
        break;

        case 65561:
        enum_num = 30;
        break;

        case 65566:
        enum_num = 31;
        break;

        case 65576:
        enum_num = 32;
        break;

        case 65586:
        enum_num = 33;
        break;

        case 65596:
        enum_num = 34;
        break;

        case 65616:
        enum_num = 35;
        break;

        case 65636:
        enum_num = 36;
        break;

        case 65661:
        enum_num = 37;
        break;

        case 65696:
        enum_num = 38;
        break;

        case 65736:
        enum_num = 39;
        break;

        case 65786:
        enum_num = 40;
        break;

        case 65856:
        enum_num = 41;
        break;

        case 65936:
        enum_num = 42;
        break;

        case 66036:
        enum_num = 43;
        break;

        case 66176:
        enum_num = 44;
        break;

        case 66336:
        enum_num = 45;
        break;

        case 66536:
        enum_num = 46;
        break;

        case 66786:
        enum_num = 47;
        break;

        case 67136:
        enum_num = 48;
        break;

        case 67536:
        enum_num = 49;
        break;

        case 68036:
        enum_num = 50;
        break;

        case 68736:
        enum_num = 51;
        break;

        case 69536:
        enum_num = 52;
        break;

        case 70536:
        enum_num = 53;
        break;

        case 71936:
        enum_num = 54;
        break;

        case 73536:
        enum_num = 55;
        break;

        default:
        enum_num = SONY_ENUM_INVALID;
        break;
    }
    return enum_num;
}

std::uint16_t enumerate_white_bal_sony_a7(std::uint32_t num) {

    std::uint16_t enum_num = 0;
    switch (num)
    {
        case 0:
        enum_num = 0;
        break;

        case 17:
        enum_num = 1;
        break;

        case 18:
        enum_num = 2;
        break;

        case 19:
        enum_num = 3;
        break;

        case 20:
        enum_num = 4;
        break;

        case 33:
        enum_num = 5;
        break;

        case 34:
        enum_num = 6;
        break;

        case 35:
        enum_num = 7;
        break;

        case 36:
        enum_num = 8;
        break;

        case 48:
        enum_num = 9;
        break;

        case 1:
        enum_num = 10;
        break;

        case 256:
        enum_num = 11;
        break;

        case 257:
        enum_num = 12;
        break;

        case 258:
        enum_num = 13;
        break;

        case 259:
        enum_num = 14;
        break;

        default:
        enum_num = SONY_ENUM_INVALID;
        break;
    }
    return enum_num;
}

#endif

