ISCNTC:
            JSR MONRDKEY
            BCC not_cntc
            cmp #3
            bne not_cntc
            jmp is_cntc
not_cntc:
            RTS
is_cntc: