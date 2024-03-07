using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Controller : MonoBehaviour {

    public bool isRightPressed;
    public bool isLeftPressed;
    public bool wasJumpPressed;

    public float moveSpeed;
    public float gravity;
    public float startJumpSpeed;
    public int maxDepth = 10;
    public LayerMask groundMask;
    [HideInInspector]
    public BoxCollider2D hitBox;
    [HideInInspector]
    public SpriteRenderer mainSprite;


    public Vector2 velocity;
    [HideInInspector]
    public Rigidbody2D rb;
    public float actualMoveSpeed;
    [HideInInspector]
    public GameObject playerSpawnPoint;
    public float maxSlopeAngle;
    public float skinWidth;
    public float minSlopeAngle;
    public float acceleration;

    public bool isGrounded = false;

    private void Awake()
    {
        rb = GetComponent<Rigidbody2D>();
        mainSprite = GetComponent<SpriteRenderer>();
        hitBox = GetComponent<BoxCollider2D>();
        
    }

	
	void Update () {

        if (isRightPressed)
        {
            mainSprite.flipX = false;
            actualMoveSpeed = Mathf.MoveTowards(actualMoveSpeed, moveSpeed, acceleration);
        }else if (isLeftPressed)
        {
            mainSprite.flipX = true;
            actualMoveSpeed = Mathf.MoveTowards(actualMoveSpeed, -moveSpeed, acceleration);
        }
        else
        {
            //not 0 to keep the sign of the previous speed
            actualMoveSpeed = Mathf.MoveTowards(actualMoveSpeed, Mathf.Sign(actualMoveSpeed)*Mathf.Epsilon, acceleration);
        }

        if (wasJumpPressed && isGrounded)
        {
            velocity.y = startJumpSpeed;
            isGrounded = false;
        }


        wasJumpPressed = Input.GetKey(KeyCode.Space);
        isRightPressed = Input.GetKey(KeyCode.D);
        isLeftPressed = Input.GetKey(KeyCode.A);
    }

    private void FixedUpdate()
    {   
        if(isGrounded) 
        {
            RaycastHit2D collision = Physics2D.BoxCast(transform.position, hitBox.size, 0, Vector2.down, Mathf.Infinity, 1<<1);
            if(collision.collider)
            {
                Debug.DrawLine(transform.position, collision.point, Color.blue);


                float maxDistance = Mathf.Max(velocity.magnitude, Mathf.Abs(actualMoveSpeed)) * Time.fixedDeltaTime;
                int direction = (int)Mathf.Sign(actualMoveSpeed);
                List<Vector2> points = GetPath(collision.point, maxDistance, collision.collider.GetComponent<CompositeCollider2D>(), direction);

                rb.MovePosition(CollideAndSlide(collision.centroid + skinWidth*Vector2.up/2, maxDistance, points));
            }
            else isGrounded = false;
            
        }
        else
        {
            velocity.y -= 9.81f*Time.fixedDeltaTime;



            float maxDistance = Mathf.Max(velocity.magnitude, Mathf.Abs(actualMoveSpeed)) * Time.fixedDeltaTime;
            rb.MovePosition(CollideAndSlide(transform.position, maxDistance, new()));
        } 



        if(transform.position.x>100) transform.position = new Vector2(-11,-0.8f);
        if(transform.position.x<-15) transform.position = new Vector2(90,2);
    }






    private Vector2 CollideAndSlide(Vector2 pos, float maxDistance, List<Vector2> keyPoints, int depth = 0)
    {
        
        //print(string.Join(", ", keyPoints));


        //0.0001 should be zero but FloatingPointPrecisionError 
        if(depth>maxDepth || (maxDistance <= 0.0001f || (Mathf.Abs(velocity.x)<=0.0001f && Mathf.Abs(velocity.y)<=0.0001f)) && depth!=0)
        {
            return pos;
        }
        
        float distance;
        Vector2 direction;
        if(depth<keyPoints.Count-1) 
        {
            Debug.DrawLine(keyPoints[depth], keyPoints[depth+1]);
            isGrounded = true;
            
            //maybe useful when modifying for use with capsules

            //Debug.DrawRay(pos + Vector2.down * (hitBox.size.y - hitBox.size.x)/2, (hitBox.size.y - hitBox.size.x)/2 * Vector2.up, Color.yellow);
            //Debug.DrawRay(pos + Vector2.down * (hitBox.size.y - hitBox.size.x)/2, -Mathf.Sign(actualMoveSpeed) * Vector2.Perpendicular(direction) * hitBox.size.x/2, Color.black);
            //Debug.DrawRay(pos, (hitBox.size.y - hitBox.size.x)/2 * Vector2.down - Mathf.Sign(actualMoveSpeed) * Vector2.Perpendicular(direction)* hitBox.size.x/2, Color.blue);
            

            //determine the side collider shifts to when on the edge
            int sign;
            if(keyPoints[depth].y - keyPoints[depth+1].y<=-0.0001f) sign = -(int)Mathf.Sign(actualMoveSpeed);
            else if(keyPoints[depth].y - keyPoints[depth+1].y>=0.0001f) sign = (int)Mathf.Sign(actualMoveSpeed);
            else if(depth+1<keyPoints.Count-1) sign = (int)(Mathf.Sign(keyPoints[depth+1].y-keyPoints[depth+2].y)*Mathf.Sign(actualMoveSpeed));
            else sign = 0;

            Vector2 closestVertexCentroid = keyPoints[depth+1] + new Vector2(sign, 1) * (hitBox.size + Vector2.one*skinWidth)/2;
            Debug.DrawLine(pos, closestVertexCentroid, Color.green); 


            direction = (keyPoints[depth+1] - keyPoints[depth]).normalized;
            if(direction == Vector2.zero) {direction = (closestVertexCentroid - pos).normalized;  }
            if(direction == Vector2.zero) {direction = velocity.normalized;}
            


            distance = Mathf.Min((closestVertexCentroid - pos).magnitude, maxDistance);
            
        }
        else 
        {
            isGrounded = false;
            
            distance = maxDistance;
            velocity.x = actualMoveSpeed;
            direction = velocity.normalized;

        }
        
        RaycastHit2D collision = Physics2D.BoxCast(pos, hitBox.size, transform.rotation.eulerAngles.z, direction, distance, 1<<1);

        if(collision.collider)
        {
            maxDistance -= (pos - collision.centroid).magnitude;


            float angle = Vector2.Angle(collision.normal, Vector2.up);

            if(angle<maxSlopeAngle && angle>minSlopeAngle)
            {
                velocity = actualMoveSpeed * -Vector2.Perpendicular(collision.normal);
            }
            else
            {
                velocity -= collision.normal*Vector2.Dot(velocity, collision.normal);
            }



            if(collision.collider.CompareTag("Ground") && angle < maxSlopeAngle) 
            {
                if(!isGrounded)
                {
                    float maxDist = Mathf.Max(velocity.magnitude, Mathf.Abs(actualMoveSpeed)) * Time.fixedDeltaTime;
                    int dir = (int)Mathf.Sign(actualMoveSpeed);
                    keyPoints = GetPath(collision.point, maxDist, collision.collider.GetComponent<CompositeCollider2D>(), dir);
                    keyPoints.InsertRange(0, new Vector2[depth+1]);
                }
                isGrounded = true;
            }



            return CollideAndSlide(collision.centroid + collision.normal*skinWidth/2, maxDistance, keyPoints, depth+1);
        }
        else 
        {
            if(depth<keyPoints.Count-1)
            {

                velocity = Mathf.Abs(actualMoveSpeed) * direction;

                return CollideAndSlide(pos + direction * distance, maxDistance - distance, keyPoints, depth+1);
            }
            else {return pos + direction*distance;}
        }
        

    }





    //returns all edges from composite collider that object will traverse when moving in specified x-direction 
    private List<Vector2> GetPath(Vector2 point, float maxDistance, CompositeCollider2D composite, int direction, int path = 0)
    {
        List<Vector2> points = new();
        if(!composite) return points;
        

        if(path<composite.pathCount) composite.GetPath(path, points);
        else return points;

        //finding the edge that point belongs to
        //0.0001f should be zero, but FloatingPointPrecisionError
        List<Vector2[]> checkX = new();
        for(int i = 0; i<points.Count-1; i++)
        {
            if((points[i].x-point.x)*(points[i+1].x-point.x)<0.0001f) checkX.Add(new Vector2[]{points[i], points[i+1]});
        }
        List<Vector2[]> checkY = new();
        foreach(Vector2[] i in checkX)
        {
            if((i[0].y-point.y)*(i[1].y-point.y)<0.0001f) checkY.Add(i);
        }
        List<Vector2> edge = null;
        foreach(Vector2[] i in checkY)
        {
            if(Vector2.Distance(i[0], point)+Vector2.Distance(i[1], point) - Vector2.Distance(i[0], i[1])<0.01f) {edge = i.ToList<Vector2>(); break;}
        }
        


        if(edge is null) return GetPath(point, maxDistance, composite, direction, path+1);

        if(direction*(edge[0].x - edge[1].x)>0) edge.Reverse();

        if(Vector2.Angle(Vector2.Perpendicular((edge[1] - edge[0])*direction), Vector2.up)>=maxSlopeAngle) return new List<Vector2>();
        
        int index = points.IndexOf(edge[1]);
        int delta = index - points.IndexOf(edge[0]);

        
        maxDistance -= Vector2.Distance(point, edge[^1]);
        do
        {
            
            index = nextIndex(index, delta, points.Count);

            if(Vector2.Angle(Vector2.Perpendicular((points[index] - edge[^1])*direction), Vector2.up)<maxSlopeAngle) edge.Add(points[index]);
            else {index -= delta; break;}

            float edgeLength = Vector2.Distance(edge[^2], edge[^1]);
            maxDistance -= edgeLength;


        }while(maxDistance>0);
        index = nextIndex(index, delta, points.Count);
        if(Vector2.Angle(Vector2.Perpendicular((points[index] - edge[^1])*direction), Vector2.up)<maxSlopeAngle) edge.Add(points[index]);
        return edge;
    }
    private int nextIndex(int index, int delta, int length)
    {
        index += delta;
        if(index == -1) index = length - 1;
        else if(index == length) index = 0;
        return index;
    }
}
