using UnityEngine;

public class BoatMovement : MonoBehaviour
{
    public float speed = 10f;
    public Vector3[] targetCoordinates;
    private int currentTarget = 0;

    private void Start()
    {
        currentTarget = 0;
    }

    private void Update()
    {
        if (currentTarget >= targetCoordinates.Length)
        {
            return;
        }

        Vector3 dir = targetCoordinates[currentTarget] - transform.position;
        transform.position = Vector3.MoveTowards(transform.position, targetCoordinates[currentTarget], speed * Time.deltaTime);
        transform.rotation = Quaternion.LookRotation(dir);

        if (Vector3.Distance(transform.position, targetCoordinates[currentTarget]) <= 0.4f)
        {
            currentTarget++;
        }
    }
}
