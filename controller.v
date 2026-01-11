/******************************************************************************
Copyright (c) 2024 SoC Design Laboratory, Konkuk University, South Korea
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met: redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer;
redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution;
neither the name of the copyright holders nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Authors: Taewoo Kim (banacbc1@konkuk.ac.kr)

Revision History
2024.11.15: Started by Taewoo Kim
*******************************************************************************/
module controller
(
        input           clk, nrst, in_vld, out_rdy,
        output wire    in_rdy, out_vld,
        output wire     we_AMEM_0,
        output wire     [4:0] addr_AMEM_0,
        output wire     we_BMEM_0,
        output wire     [4:0] addr_BMEM_0,
        output wire     we_OMEM_0,
        output wire     [4:0] addr_OMEM_0,
        output wire     we_AMEM_1,
        output wire     [4:0] addr_AMEM_1,
        output wire     we_BMEM_1,
        output wire     [4:0] addr_BMEM_1,
        output wire     we_OMEM_1,
        output wire     [4:0] addr_OMEM_1,
        output wire     [4:0] addr_CROM_0,
        output wire     [4:0] addr_CROM_1,
        output wire     en_REG_A_0,
        output wire     en_REG_B_0, en_REG_C_0,
        output wire     en_REG_A_1,
        output wire     en_REG_B_1, en_REG_C_1,
        output wire     sel_res_0,
        output wire     sel_res_1,
        output wire     sel_mem_0,
        output wire     sel_mem_1,
        output wire     sel_input,
        output wire     sel_output,
        output wire     sel_swap
);

/////////////////////////////////////
/////////// Edit code below!!////////

localparam
   stage_0 = 4'b0001,
   stage_1 = 4'b0010,
   stage_2 = 4'b0011,
   stage_3 = 4'b0100,
   stage_4 = 4'b0101,

   IDLE   = 4'b0110,        //6
   RUN   = 4'b0111,         //7                  
   STALL   = 4'b1000,       //8
   PRE_IN = 4'b1001;        //9

reg   [3:0] cstate,nstate;                    //stage상태
reg   [3:0] in_cstate, in_nstate;             //input stage 상태
reg   [3:0] out_cstate, out_nstate;           //output stage 상태

reg [6:0] cnt;                              //stage를 위한 count
reg [6:0] cnt_out;                          //출력을 위한 count
reg [6:0] cnt_in;                           //입력을 위한 count

always@(posedge clk) begin                  //stage counter
   if(!nrst) begin
      cnt <= 0;
   end   
   else begin
      if(cstate == IDLE || cstate == STALL) begin
         cnt <= 0;
      end
       else if((cstate==stage_0) && cnt == 37) begin                             
         cnt <= 0;
      end
      else if((cstate==stage_1) && cnt == 34) begin                              
         cnt <= 0;
      end
      else if((cstate==stage_2) && cnt == 34) begin                              
         cnt <= 0;
      end
      else if((cstate==stage_3) && cnt == 35) begin
           cnt <=0;
       end  
      else if((cstate==stage_4) && cnt == 34) begin
           cnt <=0;
       end       
      else begin
         cnt <= cnt + 1;
      end
   end
end

always@(posedge clk) begin                  //input counter
   if(!nrst) begin
      cnt_in <= 0;
   end   
   else begin
      if(in_cstate == IDLE || in_cstate == PRE_IN) begin    //what mean?
         cnt_in <= 0;
      end
      else if(in_vld == 1) begin            //써도 된다는 신호가오면
         cnt_in <= cnt_in + 1;            //카운터 증가해가며 쓰기
      end
      else begin
         cnt_in <= cnt_in;                //아니면 카운터 반복(출력 못하고있는거임)
      end
   end
end

always@(posedge clk) begin                  //out에 관한 counter
   if(!nrst) begin
      cnt_out <= 0;
   end   
   else begin
      if(out_cstate == IDLE) begin          
         cnt_out <= 0;
      end
      else if(out_rdy == 1) begin       //out쓰라는 신호가오면
         cnt_out <= cnt_out + 1;
      end
      else begin
         cnt_out <= cnt_out;          //신호없으면 제자리 머물기 (출력 못하고있는거임))
      end
   end
end


always @(posedge clk, negedge nrst)     //next state를 넣어주는 역할
begin
    if(!nrst) begin
       cstate <= IDLE;
      //in_cstate <= RUN;
      in_cstate <= PRE_IN;
      out_cstate <= IDLE;
    end

    else begin
       cstate <= nstate;
      in_cstate <= in_nstate;
      out_cstate <= out_nstate;
    end
end

always @(*) begin                               //input state table               
    if(nrst) begin
      case(in_cstate)                         //input 현재상태
           PRE_IN : begin
               if(nrst) begin
                  in_nstate <= RUN;           //RUN으로 바꾸기
               end
               else begin
                  in_nstate <= PRE_IN;       //리셋 신호있으면 아니면 제자리 머무르기
               end
           end
         RUN : begin
            if(cnt_in == 63 && in_vld == 1) begin   //만약 in counter가 15이고 (input 16개 전부)이거나 in_vld(넣어도 된다고하면)               ///62로 바꿔보기
               in_nstate <= IDLE;                   //돌아가기(너네 할거 다했어)
            end
            else begin
               in_nstate <= RUN;                  //16개 전부 다 넣지 않았거나 in_vld가 넣지 말라고하면 대기
            end
         end
         IDLE : begin
            if(cstate == stage_3 && cnt ==35) begin     //현재상태가 stage3이고 cnt가 7이면 run해서 input넣기
               in_nstate <= RUN;
            end
            else begin
               in_nstate <= IDLE;   
            end
         end
         default : in_nstate <= RUN;
      endcase
   end
   else begin   
   in_nstate <= in_cstate;
   end
end


always @(*) begin                                   //out_state 관련 코드
    if(nrst) begin
      case(out_cstate)
         IDLE : begin
            if(cstate == stage_4 && nstate == IDLE) begin   //stage3이고 next stage가 idle이면 
               out_nstate <= RUN;                           //실행하기 (다음에 출력할것)
            end
            else begin
               out_nstate <= IDLE;                        // 위 조건 아니면 실행 안함
            end
         end
         RUN  : begin
            if(cnt_out == 64 && out_rdy == 1) begin         //16개이고 출력할게 있으면
               out_nstate <= IDLE;                        //돌아가기
            end
            else begin
               out_nstate <= RUN;                           //아니면 계속 출력해
            end
         end
         default : out_nstate <= IDLE;
      endcase
   end
   else begin
      out_nstate <= out_cstate;
   end
end

always @(*) begin                               //main stage 코드
   if(nrst) begin
      case(cstate)
         IDLE : begin 
            if(cnt_in==63) begin
            nstate <= stage_0;   
         end
         else begin
            nstate <= IDLE;
         end
         end
         stage_0  : begin
            if(cnt == 37) begin
               nstate <= stage_1;
            end
            else begin
               nstate <= stage_0;   
            end
         end
         stage_1 : begin
            if(cnt == 34) begin
               nstate <= stage_2;
            end
            else begin
               nstate <= stage_1;
            end
         end
         stage_2 : begin
            if(cnt == 34 && out_cstate == IDLE) begin               //what mean?
               nstate <= stage_3;
            end
            else if(cnt == 34 && out_cstate == RUN) begin           //what mean?? stage2인데 out이 RUN될수가 있나?
               nstate <= STALL;
            end
            else begin
               nstate <= stage_2;
            end
         end
         stage_3 : begin
            if(cnt == 35 && out_cstate == IDLE) begin
               nstate <= stage_4;
            end
            else begin
               nstate <= stage_3;
            end
         end
         stage_4 : begin
            if(cnt == 34) begin               //what mean?
               nstate <= IDLE;
            end
            else begin
            nstate <= stage_4;
            end
         end       
         STALL : begin                           
            if(out_cstate == IDLE) begin        //out이 쉬고있으면
               nstate <= stage_4;
            end
            else begin
               nstate <= STALL;
            end
         end
         default : nstate <= IDLE;
      endcase
   end
   else begin
      nstate <= cstate;
   end
end

assign en_REG_A_0 = cstate==IDLE ?  ~cnt_in[0] :  (cstate==stage_0||(cstate==stage_4&&cnt>32))? ~cnt[0]:cnt[0];
assign en_REG_A_1 = cstate==IDLE ? cnt_in[0] : ((cstate==stage_0&& cnt>2)||cstate==stage_3)? ~cnt[0]: cnt[0];
assign en_REG_B_0 = cstate==IDLE ?  cnt_in[0] : (cstate==stage_0||(cstate==stage_4&&cnt>32))? cnt[0]:~cnt[0];
assign en_REG_B_1 = cstate==IDLE ? ~cnt_in[0] : ((cstate==stage_0 && cnt>3)||cstate==stage_3)? cnt[0]:~cnt[0];
assign en_REG_C_0 = cstate==IDLE ?  cnt_in[0] : (cstate==stage_0 && cnt>2)? ~cnt[0]:cnt[0];
assign en_REG_C_1 = cstate==IDLE ? cnt_in[0] : ((cstate==stage_0 && cnt>2)||cstate==stage_3)? ~cnt[0]:cnt[0];



reg    [4:0] addr_CROM_3b;           

always @(posedge clk)
begin
   if(!nrst) begin
      addr_CROM_3b = 0;
   end
       else if(cstate != STALL) begin
           if(!cnt[0]) begin                      //짝수마다 반복하기
                  if(cnt<2) begin               //초반 2cycle은 input 2개?
                        addr_CROM_3b = 0;         // 그냥 W가 0이기에
                  end
                  else begin
                         addr_CROM_3b =(addr_CROM_3b + (6'b100000 >> cstate)); //stage0:0->0,0,0,, stage1->4,0,4 stage2->2,4,6,0,2,4,6 stage3:3 -> 0,1,2,3,4,5,6,7
                  end
           end
   end 
        else begin                              //특수한 상황 (STALL) 이면 멈추기
            addr_CROM_3b <= addr_CROM_3b ;
        end
end

assign addr_CROM_0 = (cstate==5&&cnt>31) ? 0:(cstate==6 &&out_vld==0)? 0: (cstate==stage_0 ? (cnt<=3?0:({!addr_CROM_3b[4], addr_CROM_3b[3],addr_CROM_3b[2], addr_CROM_3b[1], addr_CROM_3b[0]})) : {addr_CROM_3b[4], addr_CROM_3b[3], addr_CROM_3b[2], addr_CROM_3b[1], addr_CROM_3b[0]});
assign addr_CROM_1 = (cstate==stage_0||cstate==IDLE)?  (cnt<=3?0:({!addr_CROM_3b[4], addr_CROM_3b[3],addr_CROM_3b[2], addr_CROM_3b[1], addr_CROM_3b[0]})) : cstate==stage_4 ? {addr_CROM_3b[4], addr_CROM_3b[3],addr_CROM_3b[2], addr_CROM_3b[1], addr_CROM_3b[0]}+16:{addr_CROM_3b[4], addr_CROM_3b[3],addr_CROM_3b[2], addr_CROM_3b[1], addr_CROM_3b[0]}; //stage_5로 고침

assign in_rdy = ((in_cstate == RUN) && (nrst==1)) ? 1 : 0;            
assign out_vld = ((cnt_out > 0) && (cnt_out < 65)) ? 1 : 0;
assign sel_input = in_rdy;

wire   [4:0] addr_rd, addr_rd0, addr_wr ,addr_wr0, addr_rd2;
wire   [4:0] addr_rd_1, addr_wr1,addr_wr2;

assign addr_rd = ((cnt >> (cstate + 1)) << cstate) + (cnt >> 1) + (cnt[0] << cstate);   
assign addr_rd0 = (((cnt-3) >> (cstate + 1)) << cstate) + ((cnt-3) >> 1) + (!cnt[0] << cstate);   

assign addr_rd_1 = ((cnt >> (4 + 1)) << 2) + (cnt >> 1) + (cnt[0] << 4);     
assign addr_rd2 = cstate==stage_3 ? ((cnt-1 >> (cstate + 1)) << cstate) + (cnt-1 >> 1) + (~cnt[0] << cstate) : ((cnt >> (cstate + 1)) << cstate) + (cnt >> 1) + (cnt[0] << cstate);
              
assign addr_wr = (((cnt-3) >> (cstate + 1)) << cstate) + ((cnt-3) >> 1) + ((!cnt[0]) << cstate);        // 그냥 stage mem
assign addr_wr0 = (((cnt-6) >> (cstate + 1)) << cstate) + ((cnt-6) >> 1) + ((cnt[0]) << cstate);      //stage 1 Amem input  
    
assign addr_wr1 = (((cnt-3) >> (4 + 1)) << 2) + ((cnt-3) >> 1) + ((!cnt[0]) << 4);        
assign addr_wr2 = cstate==stage_3 ? (((cnt-4) >> (cstate + 1)) << cstate) + ((cnt-4) >> 1) + ((cnt[0]) << cstate)  : (((cnt-3) >> (cstate + 1)) << cstate) + ((cnt-3) >> 1) + ((!cnt[0]) << cstate);  //수정

assign we_AMEM_0    = ((in_cstate == RUN && ~cnt_in[0]) || (cstate == stage_0 && cnt > 5) || (cstate == stage_2 && cnt > 2) ) ? 0 : 1;   //stage3조건 추가
assign we_AMEM_1    = ((in_cstate == RUN && cnt_in[0]) || (cstate == stage_0 && cnt > 5)|| (cstate == stage_2 && cnt > 2)) ? 0 : 1;   //stage3조건 추가
assign addr_AMEM_0 = we_AMEM_0 ? (cstate==IDLE ||cstate==stage_4) ?  {cnt_in[1], cnt_in[2], cnt_in[3], cnt_in[4], ~cnt_in[5]}: addr_rd:(in_rdy ? {cnt_in[1], cnt_in[2], cnt_in[3], cnt_in[4], cnt_in[5]} : cstate== stage_0? addr_wr0:addr_wr);        
assign addr_AMEM_1 = we_AMEM_1 ? (cstate==IDLE ||cstate==stage_4)? {cnt_in[1], cnt_in[2], cnt_in[3], cnt_in[4], ~cnt_in[5]} : addr_rd2 : (in_rdy ? {cnt_in[1], cnt_in[2], cnt_in[3], cnt_in[4], cnt_in[5]} : cstate== stage_0? addr_wr0:addr_wr);         
reg [5:0] cnt2;
wire [4:0] cnt3;

always @(posedge clk) begin
    cnt2 <= cnt_in-2;
    end
assign cnt3 = cnt_in[0] ? {cnt2[1], cnt2[2], cnt2[3], cnt2[4], ~cnt2[5]} :{cnt2[1], cnt2[2], cnt2[3], cnt2[4], cnt2[5]};

assign we_BMEM_0    = ((cstate == IDLE) || (cstate == stage_1 && cnt > 2) || (cstate == stage_3 && cnt > 2 && cnt < 35)||(cstate==stage_0 &&cnt<3)) ? 0 : 1; //stage4추가
assign we_BMEM_1    = ((cstate == IDLE) || (cstate == stage_1 && cnt > 2) || (cstate == stage_3 && cnt > 3 && cnt < 36)||(cstate==stage_0 &&cnt<3)) ? 0 : 1; //stage4 추가
assign addr_BMEM_0 = we_BMEM_0 ?  (cstate == stage_4?addr_rd_1:cstate == stage_0 ? addr_rd0:addr_rd): cstate==IDLE? cnt3: cstate == stage_0? (cnt==0? 15 : cnt==1? 30 : addr_wr): addr_wr;      //쓰레기 코드.
assign addr_BMEM_1 = we_BMEM_1 ?  (cstate == stage_4?addr_rd_1:cstate == stage_0 ? addr_rd0:addr_rd): cstate==IDLE? cnt3:cstate == stage_0? (cnt==0? 15 : cnt==1? 30 : addr_wr): addr_wr2;      //쓰레기 코드 2

assign we_OMEM_0    = ((cstate == stage_4) && cnt > 2) ? 0 : 1;
assign we_OMEM_1    = ((cstate == stage_4) && cnt > 2) ? 0 : 1;
assign addr_OMEM_0 = we_OMEM_0 ? (out_rdy ? (cnt_out[5] ? cnt_out[5:0]-16:cnt_out[5:0]) : (cnt_out[5:0] - 1'b1)):addr_wr1;
assign addr_OMEM_1 = we_OMEM_1 ? (out_rdy ? (cnt_out[5] ? cnt_out[5:0]-32:cnt_out[5:0]-16) : (cnt_out[5:0] - 1'b1)):addr_wr1;

assign sel_res_0 = en_REG_C_0;
assign sel_res_1 = en_REG_C_1;

assign sel_mem_0  = (cnt_in>32) ? 0 :cstate[0];
assign sel_mem_1  =(cnt_in>32) ? 0 :cstate[0];

assign sel_output = ~((cnt_out-1)>>4);
assign sel_swap = (cstate==stage_3 &&~cnt[0])? 1:0;

//////////Edit code above!!/////////
////////////////////////////////////      
      
endmodule