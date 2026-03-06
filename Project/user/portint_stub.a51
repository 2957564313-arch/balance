; =========================
; File: portint_stub.a51
; 把 P0/P3 端口中断扩展向量转发到 USER_VECTOR(13) 的标准向量 006BH
; =========================

CSEG    AT      012BH      ; P0INT_VECTOR (37) -> 012BH
        LJMP    006BH      ; USER_VECTOR (13) -> 006BH

CSEG    AT      0143H      ; P3INT_VECTOR (40) -> 0143H
        LJMP    006BH      ; USER_VECTOR (13) -> 006BH

END